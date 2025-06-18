#include <SHT2x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <Update.h>
#include "config.h"/
#include "SD_MMC.h" 
#include <SPIFFS.h>
#include <FS.h>
#include <SPI.h>
#include "driver/rtc_io.h"
#include <soc/sens_reg.h>
#include <driver/adc.h>
#include <BH1750.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ThingsBoard.h>
#include <Espressif_Updater.h>
#include <Arduino_MQTT_Client.h>

// WiFi connection parameters
#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"
const char* host = "Bird Feeder";

// Statuses for updating
Espressif_Updater updater;
constexpr char CURRENT_FIRMWARE_TITLE[] = "birdfeeder_code";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.1.1";
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 48U;
constexpr uint16_t FIRMWARE_PACKET_SIZE = 32768U;
bool currentFWSent = false;
bool updateRequestSent = false;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

IPAddress local_IP(192, 168, 0, 115);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 0, 1);
IPAddress secondaryDNS(0, 0, 0, 0);

TwoWire I2C = TwoWire(0);
Adafruit_BME280 bme;
BH1750 lightMeter;
WiFiClient client;

//Thingsboard parameters
constexpr char THINGSBOARD_SERVER[] = "192.168.0.241";
constexpr char TOKEN[] = "AqpgGhYEjgeJm8iCSMVF";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SIZE = 256U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
int status = WL_IDLE_STATUS;
bool tbStatus = false;
bool wifiStatus = false;

// Initalize the Mqtt client instance
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);

// Initialize ThingsBoard instance
ThingsBoardSized<256> tb(mqttClient, MAX_MESSAGE_SIZE);

// Measurement parameters
int batteryLevel;
int waterLevel;
float airPressure;
float airTemperature;
float airHumidity;
float solarIntensity;
bool rain;
bool motionTrigger = false;

// Registry parameters
uint64_t reg_a;
uint64_t reg_b;
uint64_t reg_c;
char buffer [50];

// Pin Layout
int I2C_SDA = 21;
int I2C_SCL = 22;
int BATTERY_VOLTAGE_PIN = 4;
int MOTION_SENSOR_PIN = 12;
int WATER_SENSOR_PIN = 20;
int RAIN_SENSOR_PIN = 14;
int INDICATOR_LED_PIN = 2;

// Stored parameters
RTC_DATA_ATTR int upTime = 0;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int sleepTime = 10;
RTC_DATA_ATTR bool devMode = false;
RTC_DATA_ATTR double tempCalibration = 0;
RTC_DATA_ATTR double presCalibration = 0;
RTC_DATA_ATTR double humCalibration = 0;

// Statuses for subscribing to shared attributes
bool subscribed = false;

// Statuses for requesting of attributes
bool requestedClient = false;
bool requestedShared = false;

// Processes function for RPC call "settings"
RPC_Response settings(const RPC_Data &data) {

  // Received settings input
  String inputValue = data.as<String>();
  Serial.print("Settings input received: ");
  Serial.println(inputValue);
  
  // Read and split settings input
  int verb=getCommands(data,'#',0).toInt();
  int noun=getCommands(data,'#',1).toInt();
  int action=getCommands(data,'#',2).toInt();

  Serial.print("    Verb: ");
  Serial.println(verb);
  Serial.print("    Noun: ");
  Serial.println(noun);
  Serial.print("    Action: ");
  Serial.println(action); 
};

// RPC handlers
const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "settings",         settings }
};

void processSharedAttributeRequest(const Shared_Attribute_Data &data) {

  Serial.println("------------ SHARED ATTRIBUTES ------------");
  for (auto it = data.begin(); it != data.end(); ++it) {
    //Set boot parameter
    setupBootParameters(it->key().c_str(),it->value().as<double>());
  }

  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
  Serial.println("------------ SHARED ATTRIBUTES ------------");
}

void processClientAttributeRequest(const Shared_Attribute_Data &data) {
  
  Serial.println("------------ CLIENT ATTRIBUTES ------------");
  for (auto it = data.begin(); it != data.end(); ++it) {
    //Set boot parameter
    setupBootParameters(it->key().c_str(),it->value().as<double>());
  }

  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
  Serial.println("------------ CLIENT ATTRIBUTES ------------");
}

/// Updated callback that will be called as soon as the firmware update finishes
void updatedCallback(const bool& success) {
  if (success) 
  {
    consoleLog("Update Done, Reboot now");

    devMode = false;
    tb.sendAttributeData("devMode",devMode);
    
    // Turn off WiFi connection
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect();
    WRITE_PERI_REG(SENS_SAR_START_FORCE_REG, reg_a);  // fix ADC registers
    WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
    WRITE_PERI_REG(SENS_SAR_MEAS_START2_REG, reg_c);
    esp_restart();

    return;
  }
  consoleLog("Downloading firmware failed");
}

/// Progress callback that will be called every time we downloaded a new chunk successfully
void progressCallback(const size_t& currentChunk, const size_t& totalChuncks) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(currentChunk * 100U) / totalChuncks);
}

// Method to print the reason by which ESP32
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : 
      Serial.println("     Wakeup caused by external signal using RTC_IO");
      //Motion sensor triggered, take picture
      motionTrigger = true;
      break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
      Serial.println("     Wakeup caused by external signal using RTC_CNTL");
      //Motion sensor triggered, take picture
      motionTrigger = true;
      break;
    case ESP_SLEEP_WAKEUP_TIMER : 
      Serial.println("      Wakeup caused by timer"); 
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : 
      Serial.println("     Wakeup caused by touchpad"); 
      break;
    case ESP_SLEEP_WAKEUP_ULP :
      Serial.println("      Wakeup caused by ULP program"); 
      break;
    default : 
      Serial.printf("     Wakeup was not caused by deep sleep: %d\n",wakeup_reason); 
      break;
  }
}

// Reads battery level
int readBatteryLevel(int pin, int sumNumber) {

    int sum = 0;
    
    for (int i = 0; i <= sumNumber; i++) {
      sum += analogRead(pin);
      delay(100);
    }
    
    return sum/sumNumber;
}

// Reads water level
int readWaterLevel(int pin, int sumNumber) {

    int sum = 0;

    float value;
    
    for (int i = 0; i <= sumNumber; i++) {
      sum += analogRead(pin);
      delay(100);
    } 

    return sum/sumNumber;
}

// Function to connect wifi
bool connectWifi()
{
  int attempts = 1;
  
  // Connect to WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    attempts = attempts + 1;
    delay(500);
    Serial.print(".");

    if(attempts == 50){
      // Failed to connect
      Serial.println("Failed to connect to WiFi");
      return false;
    }
  }
  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  return true;
}

// Function to connect Thingsboard
bool connectTB()
{
  bool connection = false;
  int attempts = 1;
  
  while (!tb.connected() && attempts < 3) 
  {
    if ( tb.connect(THINGSBOARD_SERVER, TOKEN) ) {
      Serial.println("Succesfully connected to TB server!");

      if (!subscribed){
        Serial.println("Subscribing for shared attribute updates");

        if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) 
        {
          Serial.println("[ERROR] Failed to subscribe for RPC");
        }
        else {
          Serial.println("Subscribe done");
          subscribed = true;  
        }
      }

      connection = true;
    } 
    else {
      Serial.println("Failed to connected to TB server (No: " + String(attempts) + ")");
      attempts = attempts + 1;
      subscribed = false;
      delay( 1000 );
    }
  }

  return connection;
}

void consoleLog(String consoleText)
{
    tb.sendTelemetryData("console", consoleText.c_str());
    Serial.println(consoleText);
}

void setup() {

  if (!devMode) 
  {
    Serial.begin(115200);
  
    // Disable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
    // Save register values for the WiFi setting
    reg_a = READ_PERI_REG(SENS_SAR_START_FORCE_REG);
    reg_b = READ_PERI_REG(SENS_SAR_READ_CTRL2_REG);
    reg_c = READ_PERI_REG(SENS_SAR_MEAS_START2_REG);
  
    // Turn on indicator LED
    pinMode(INDICATOR_LED_PIN, OUTPUT);
    digitalWrite(INDICATOR_LED_PIN,HIGH);
  
    // Configure ADC pins
    pinMode(BATTERY_VOLTAGE_PIN, INPUT);
    pinMode(WATER_SENSOR_PIN, INPUT);
    pinMode(MOTION_SENSOR_PIN, INPUT);
    adcAttachPin(BATTERY_VOLTAGE_PIN);
    adcAttachPin(WATER_SENSOR_PIN);
    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);
  
    // Take some time to open up the Serial Monitor
    delay(1000);
  
    // Enable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);
  
    // Increment boot number and print it every reboot
    bootCount++;
    Serial.println("");
    Serial.println("*******************************************************");
    Serial.println("      Boot Number: " + String(bootCount));
    Serial.println("      Uptime: " + String(upTime));
  
    // Print the wakeup reason for ESP32
    print_wakeup_reason();

    Serial.println("*******************************************************");
    Serial.println("");
    
    esp_sleep_enable_timer_wakeup(sleepTime * uS_TO_S_FACTOR);
  
    // Initialize I2C bus
    I2C.begin(I2C_SDA, I2C_SCL, 100000);
    delay(200);
    
    // Initialize BME280
    bme.begin(0x76, &I2C);
    delay(200);
    
    // Initialize light meter
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2C);
  
    // Make measurements
    Serial.println("Starting measurements");

    waterLevel = readWaterLevel(WATER_SENSOR_PIN,50);
    Serial.print("    Water Level: ");
    Serial.println(waterLevel);
    delay(200);
    
    batteryLevel = readBatteryLevel(BATTERY_VOLTAGE_PIN,50);
    Serial.print("    Battery Level: ");
    Serial.println(batteryLevel);
    delay(200);
  
    solarIntensity = lightMeter.readLightLevel();
    Serial.print("    Solar Intesity: ");
    Serial.println(solarIntensity);
    delay(200);
    
    airPressure = bme.readPressure()/100 + presCalibration;
    Serial.print("    Air Pressure: ");
    Serial.println(airPressure);
    delay(200);
    
    airTemperature = bme.readTemperature() + tempCalibration;
    Serial.print("    Air Temperature: ");
    Serial.println(airTemperature);
    delay(200);
    
    airHumidity = bme.readHumidity() + humCalibration;
    Serial.print("    Air Humidity: ");
    Serial.println(airHumidity);
    delay(200);

    rain = digitalRead(RAIN_SENSOR_PIN);
    rain = !rain;
    Serial.print("    Rain Sensor: ");
    Serial.println(rain);

    // Connect to WiFi network
    wifiStatus = connectWifi();

    // Connect to Thingsboard Server
    tbStatus = connectTB();

    //Request client side attributes
    constexpr std::array<const char*, 1U> REQUESTED_CLIENT_ATTRIBUTES = { "operationStatus"};
    requestedClient = requestClientAttributes(REQUESTED_CLIENT_ATTRIBUTES);

    //Request shared side attributes
    constexpr std::array<const char*, 5U> REQUESTED_BOOT_PARAMETERS = { "sleepTime", 
                                                                          "devMode",
                                                                          "tempCalibration",
                                                                          "presCalibration",
                                                                          "humCalibration",};
    requestedShared = requestSharedAttributes(REQUESTED_BOOT_PARAMETERS);

    // Open Thingsboard connection to receive attributes
    Serial.println("Opening connection to Thingsboard server");
    for (int i = 0; i <= 100; i++) {
      tb.loop();
      delay(50);
    }
    Serial.println("Closing connection to Thingsboard server");

    // Send data to Thingboards server
    Serial.println("-----------------------------------------");
    consoleLog("Sending data to TB server");
    delay(200);
    
    tb.sendTelemetryData("temperature", airTemperature);
    Serial.println("    Temperature data sent");
    delay(200);
    
    tb.sendTelemetryData("humidity", airHumidity);
    Serial.println("    Humidity data sent");
    delay(200);
    
    tb.sendTelemetryData("pressure", airPressure);
    Serial.println("    Pressure data sent");
    delay(200);
    
    tb.sendTelemetryData("battery", batteryLevel);
    Serial.println("    Battery data sent");
    delay(200);
    
    tb.sendTelemetryData("solar", solarIntensity);
    Serial.println("    Solar data sent");
    delay(200);
    
    tb.sendTelemetryData("uptime", upTime);
    Serial.println("    Uptime sent");
    delay(200); 
    
    tb.sendTelemetryData("water", waterLevel);
    Serial.println("    Water level sent");
    delay(200);

    tb.sendTelemetryData("rain", rain);
    Serial.println("    Rain sensor sent");
    delay(200);

    tb.sendTelemetryData("motiontrigger", motionTrigger);
    Serial.println("    Motion trigger sent");
    delay(200);

    tb.sendAttributeData("operationStatus",false);
    delay(200);
    consoleLog("Deep sleep started");

    delay(500);
  
    // Turn off indicator LED
    digitalWrite(INDICATOR_LED_PIN,LOW);
  
    // Turn off WiFi connection
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect();
    WRITE_PERI_REG(SENS_SAR_START_FORCE_REG, reg_a);  // fix ADC registers
    WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
    WRITE_PERI_REG(SENS_SAR_MEAS_START2_REG, reg_c);
  
    // Activating deep sleep
    Serial.println("*******************************************************");
    Serial.print("Deep sleep activated! (Sleep Time: ");
    Serial.print(sleepTime);
    Serial.println(" s)");
    Serial.println("*******************************************************");
    delay(500);
    
    Serial.flush(); 
    esp_deep_sleep_start();
  }
  else 
  {
    // Disable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    
    Serial.begin(115200);

    Serial.println("");
    Serial.println("*******************************************************");
    Serial.println("      DEVELOPER MODE");
    Serial.println("*******************************************************");
    Serial.println("");
    
    // Save register values for the WiFi setting
    reg_a = READ_PERI_REG(SENS_SAR_START_FORCE_REG);
    reg_b = READ_PERI_REG(SENS_SAR_READ_CTRL2_REG);
    reg_c = READ_PERI_REG(SENS_SAR_MEAS_START2_REG);

    // Take some time to stabilize
    delay(1000);

    // Enable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);
    
    // Connect to WiFi network
    connectWifi();
    
    //Initialize Thingsboard connection
    connectTB();
    
    tb.sendAttributeData("operationStatus",true);
    consoleLog("Developer mode started");
  }
}

void loop() {
  //Firmware update mode 
  tb.loop();

  if (!currentFWSent) {
    // Firmware state send at the start of the firmware
    currentFWSent = tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION) && tb.Firmware_Send_State(FW_STATE_UPDATED);
  }

  if (!updateRequestSent) {
    Serial.println("Checking for firmware update");
    const OTA_Update_Callback callback(&progressCallback, &updatedCallback, CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION, &updater, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
    updateRequestSent = tb.Start_Firmware_Update(callback);
  }

  delay(200);
}

String getCommands(String data, char separator, int index)
{
  //
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setupBootParameters(String parameter, double value){

  if (parameter == "sleepTime") {
      sleepTime = value;
      Serial.print("    [SET] sleepTime");
      Serial.print(" [TO] ");
      Serial.println(value);

  } else if (parameter == "devMode") {
      devMode = value;
      Serial.print("    [SET] devMode");
      Serial.print(" [TO] ");
      Serial.println(value);

  } else if (parameter == "tempCalibration") {
      tempCalibration = value;
      Serial.print("    [SET] tempCalibration");
      Serial.print(" [TO] ");
      Serial.println(value);

  } else if (parameter == "presCalibration") {
      presCalibration = value;
      Serial.print("    [SET] presCalibration");
      Serial.print(" [TO] ");
      Serial.println(value);

  } else if (parameter == "humCalibration") {
      humCalibration = value;
      Serial.print("    [SET] humCalibration");
      Serial.print(" [TO] ");
      Serial.println(value);

  } else {
    Serial.println("Invalid boot parameter");
  }
}

template<size_t M>
bool requestSharedAttributes (const std::array<const char*, M>& REQUESTED_SHARED_ATTRIBUTES){
  
  Serial.println("Requesting shared attributes...");

  bool success = false;

  const Attribute_Request_Callback sharedCallback(&processSharedAttributeRequest, REQUESTED_SHARED_ATTRIBUTES.cbegin(), REQUESTED_SHARED_ATTRIBUTES.cend());
  success = tb.Shared_Attributes_Request(sharedCallback);

  if (!success) {
    Serial.println("Failed to request shared attributes");
  }

  return success;
}

template<size_t N>
bool requestClientAttributes (const std::array<const char*, N>& REQUESTED_CLIENT_ATTRIBUTES){
  
  Serial.println("Requesting client-side attributes...");

  bool success = false;

  const Attribute_Request_Callback clientCallback(&processClientAttributeRequest, REQUESTED_CLIENT_ATTRIBUTES.cbegin(), REQUESTED_CLIENT_ATTRIBUTES.cend());
  success = tb.Client_Attributes_Request(clientCallback);

  if (!success) {
    Serial.println("Failed to request shared attributes");
  }

  return success;
}
