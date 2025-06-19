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

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "weatherstation";
const char* currentSwVersion = "1.0.1";
const char* deviceModel = "ESP32-NodeMCU";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";
String  firmwareUrl;
String  latestSwVersion;
bool debugMode = false;

// WiFi connection parameters
#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"
const char* host = "Bird Feeder";


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

// Initalize the Mqtt client instance
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);

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

/************************** HomeAssistant Settings ***********************************/

// MQTT broker details
const char* mqtt_server = "192.168.0.241";
const int mqtt_port = 1783;
const char* mqtt_client_id = deviceName;
bool mqttStatus = false;

// MQTT topics
// Diagnostic
String ota_query_topic =                    String("home/ota/query");
String parameter_request_topic =            String("home/parameters/request");
String availability_topic =                 String("home/") + deviceName + String("/available");
String parameter_response_topic =           String("home/") + deviceName + String("/parameters");
String ota_status_topic =                   String("home/") + deviceName + String("/ota/status");
String ota_response_topic =                 String("home/") + deviceName + String("/ota/response");
String log_info_topic =                     String("home/") + deviceName + String("/log/info");
String log_warning_topic =                  String("home/") + deviceName + String("/log/warning");
String log_error_topic =                    String("home/") + deviceName + String("/log/error");
String command_topic =                      String("home/") + deviceName + String("/command");
String uptime_topic =                       String("home/") + deviceName + String("/uptime");
String firmware_topic =                     String("home/") + deviceName + String("/firmware");
String ip_topic =                           String("home/") + deviceName + String("/ip");
String wifi_strength_topic =                String("home/") + deviceName + String("/wifi/strength");

// Sensors
String pump1_topic =                        String("home/") + deviceName + String("/pumps/pump1");
String pump2_topic =                        String("home/") + deviceName + String("/pumps/pump2");
String led_topic =                          String("home/") + deviceName + String("/led");
String IRsensor_topic =                     String("home/") + deviceName + String("/IRsensor");
String USsensor_topic =                     String("home/") + deviceName + String("/USsensor");
String soil1_topic =                        String("home/") + deviceName + String("/moisture/soil1");
String soil2_topic =                        String("home/") + deviceName + String("/moisture/soil2");
String soil3_topic =                        String("home/") + deviceName + String("/moisture/soil3");

// Parameters
String refreshRate_topic =                  String("home/") + deviceName + String("/parameters/refreshRate");

// Initalize the Mqtt client instance
WiFiClient espClient;
PubSubClient client(espClient);

// Connect to the predefined MQTT broker
bool connectMQTT()
{
  int attempts = 1;
  
  Serial.println("Connecting to MQTT server:");
  
  while (!client.connected() && attempts < 3) 
  {
    if ( client.connect(mqtt_client_id) ) {

      // Subscribe to command topics
      subscribeTopic(ota_response_topic);
      subscribeTopic(command_topic);
      subscribeTopic(parameter_response_topic);

      // Send discovery payload
	    sendDiscoveries();

      Serial.println("	[DONE]");
      mqttStatus = true;
    } 
    else {
      Serial.println("	[ERROR] Failed to connected to TB server (No: " + String(attempts) + ")");
      attempts = attempts + 1;
      mqttStatus = false;
	  
      delay(1000);
    }
  }

  return mqttStatus;
}

// MQTT Callback Function
void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Received message on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  // Handle commands
  if (String(topic) == ota_response_topic) 
  {
    updateFirmwareOTA(message);
  }
  else if (String(topic) == parameter_response_topic) 
  {
    handleParameters(message);
  }
  else if (String(topic) == command_topic) 
  {
    handleCommands(message);
  }
  else
  {
    consoleLog("Unknown MQTT message received", 2);
  }
}

// Subscribe to an MQTT topic
void subscribeTopic(String topic)
{
    client.subscribe(topic.c_str());
    Serial.print("	Subscribed to: ");
    Serial.println(topic);
}

// Publish an MQTT message with an int payload
void publishMessage(String topic, int payload, bool retain) 
{
  char message[16]; // Buffer to store the stringified payload
  itoa(payload, message, 10); // Convert int to string
  if (client.publish(topic.c_str(), message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a double payload
void publishMessage(String topic, double payload, bool retain) 
{
  char message[16]; // Buffer to store the stringified payload
  dtostrf(payload, 1, 2, message); // Convert double to string with 2 decimal places
  if (client.publish(topic.c_str(), message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a boolean payload
void publishMessage(String topic, bool payload, bool retain) 
{
  const char* message = payload ? "true" : "false"; // Convert bool to "true"/"false"
  if (client.publish(topic.c_str(), message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a String payload
void publishMessage(String topic, String payload, bool retain) 
{
  const char* message = payload.c_str();
  if (client.publish(topic.c_str(), message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a const char* payload
void publishMessage(String topic, const char* payload, bool retain) 
{
  if (client.publish(topic.c_str(), payload, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Read and format OTA response message
void parseOTAResponse(const String& response) 
{
  StaticJsonDocument<256> doc;

  // Deserialize the JSON
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    consoleLog("OTA update JSON deserialization failed ", 3);
    Serial.println(error.c_str());
    return;
  }

  // Extract values
  const char* device = doc["device"].as<const char*>();
  latestSwVersion =doc["version"].as<const char*>();
  firmwareUrl = doc["url"].as<const char*>();

  // Debugging output
  Serial.println("Parsed OTA Response:");
  Serial.print("    Device: ");
  Serial.println(device);
  Serial.print("    Version: ");
  Serial.println(latestSwVersion);
  Serial.print("    URL: ");
  Serial.println(firmwareUrl);
}

// Handle OTA firmware update
void performOTA() 
{
  consoleLog("Starting OTA update process", 1);
  publishMessage(ota_status_topic.c_str(), "Updating", true);
  
  // Resource optimization: Check heap memory
  if (ESP.getFreeHeap() < 20000) {
    consoleLog("Not enough memory for OTA update", 3);
    publishMessage(ota_status_topic.c_str(), "Failed", true);
    return;
  }

  HTTPClient http;

  http.setTimeout(10000); // Set 10-second timeout

  http.begin(firmwareUrl);
  int httpCode = http.GET();

  if (httpCode > 0) {
    Serial.printf("[INFO] HTTP response code: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK) {
      Serial.println("[INFO] HTTP request successful!");
    } else {
      Serial.printf("[ERROR] Unexpected HTTP response code: %d\n", httpCode);
      publishMessage(ota_status_topic.c_str(), "Failed", true);
    }
  } else {
    Serial.printf("[ERROR] HTTP request failed with code: %d\n", httpCode);
    Serial.printf("[ERROR] HTTP error: %s\n", http.errorToString(httpCode).c_str());
    publishMessage(ota_status_topic.c_str(), "Failed", true);
  }

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    WiFiClient* stream = http.getStreamPtr();

    if (Update.begin(contentLength)) {
      size_t written = Update.writeStream(*stream);

      if (written == contentLength && Update.end()) {
        consoleLog("OTA update successful!", 1);
        Serial.print("		From: ");
        Serial.println(currentSwVersion);
        Serial.print("		To: ");
        Serial.println(latestSwVersion);
        Serial.print("		Size: ");
        Serial.println(written);
        consoleLog("Restarting device in 10 seconds", 1);
        publishMessage(ota_status_topic.c_str(), "Updated", true);
        delay(10000);
        ESP.restart();
      } else {
        consoleLog("OTA update failed", 3);
        Update.printError(Serial);
        publishMessage(ota_status_topic.c_str(), "Failed", true);
      }
    } else {
      consoleLog("Not enough space for OTA update", 3);
      publishMessage(ota_status_topic.c_str(), "Failed", true);
    }
  } else {
    Serial.printf("[ERROR] HTTP request failed with code: %d\n", httpCode);
    consoleLog("OTA update failed", 3);
    publishMessage(ota_status_topic.c_str(), "Failed", true);
  }

  http.end();
}

// Request latest firmware version from Home Assistant
void requestFirmwareVersion() 
{
  String queryPayload = String("{\"device\":\"") + deviceName + String("\",\"version\":\"") + currentSwVersion + String("\"}");
  publishMessage(ota_query_topic, queryPayload.c_str(),false);
}

// Check if device has the latest firmware
void updateFirmwareOTA(String msg)
{
  parseOTAResponse(msg);

  Serial.println("[INFO] Checking for firmware update");
  Serial.print("    Current version: ");
  Serial.println(currentSwVersion);
  Serial.print("    Latest version: ");
  Serial.println(latestSwVersion);

  // Compare versions
  if (latestSwVersion != currentSwVersion) {
    consoleLog("Firmware update available! Starting OTA update", 1);
    performOTA();
  } else {
    consoleLog("Device firmware is up to date.", 1);
    publishMessage(ota_status_topic.c_str(), "Up to date", true);
  }
}

// Request server-side parameters from Home Assistant
void requestParameters()
{
  String requestPayload = String("{\"device\":\"") + deviceName + String("\"}");
  Serial.printf("[INFO] Publishing parameter request to MQTT: %s\n", requestPayload.c_str());
  client.publish(parameter_request_topic.c_str(), requestPayload.c_str());
}

// Remove white spaces from a String input
String removeSpaces(String input) 
{
  String output = "";
  for (int i = 0; i < input.length(); i++) {
    if (input[i] != ' ') {  
      output += input[i];  // Append only non-space characters
    }
  }
  return output;
}

// Create sensor discovery payload
void publishMQTTSensorDiscovery(String name, String deviceType,	String icon, String unitOfMeasurement, String deviceClass, String stateClass, String entityCategory, String stateTopic, int displayPrecision) 
{
    // Construct IDs
    String uniqueID = String(deviceName) + "-" + removeSpaces(name);
    String objectID = String(deviceName) + "_" + removeSpaces(name);
    
    // Construct discovery topic
    String topic = "homeassistant/" + deviceType + "/" + uniqueID + "/config";
    
    // Create a JSON document
    DynamicJsonDocument doc(1024);  // Adjust the size if necessary

    // Fill the JSON document with values
    doc["name"] = name;
    doc["unique_id"] = uniqueID;
    doc["object_id"] = objectID;
    doc["icon"] = icon;
    doc["state_topic"] = stateTopic;
    if(entityCategory != "")
    {
      doc["entity_category"] = entityCategory; 
    }
    if(unitOfMeasurement != "")
    {
      doc["unit_of_meas"] = unitOfMeasurement; 
    }
    if(deviceClass != "")
    {
      doc["device_class"] = deviceClass; 
    }
    if(stateClass != "")
    {
      doc["state_class"] = stateClass; 
    }
    if(displayPrecision != -1)
    {
      doc["suggested_display_precision"] = displayPrecision; 
    }
    doc["availability_topic"] = availability_topic;
    doc["payload_available"] = "connected";
    doc["payload_not_available"] = "connection lost";
    
    // Add the device object
    JsonObject device = doc.createNestedObject("device");
    JsonArray identifiers = device.createNestedArray("identifiers");
    identifiers.add(deviceName);

    device["name"] = deviceName;
    device["model"] = deviceModel;
    device["manufacturer"] = deviceManufacturer;
    // device["configuration_url"] = configurationUrl;
    // device["sw_version"] = currentSwVersion;

    // Serialize the JSON document to a string
    String payload;

    if (serializeJson(doc, payload) == 0) {
        Serial.println("Failed to serialize JSON!");
        return;
    }
    
    // Publish discovery message
    publishMessage(topic.c_str(), payload.c_str(), true);
}

// Send discovery topics to Home Assistant
void sendDiscoveries()
{
  // Diagnostics
  publishMQTTSensorDiscovery("Up Time", "sensor","mdi:clock", "h", "duration", "total_increasing", "diagnostic", uptime_topic, 0);
  delay(100);
	publishMQTTSensorDiscovery("OTA Status", "sensor", "mdi:update", "", "", "", "diagnostic", ota_status_topic, -1);
  delay(100);
	publishMQTTSensorDiscovery("Firmware Version", "sensor", "mdi:application-outline", "", "", "", "diagnostic", firmware_topic, -1);
  delay(100);
	publishMQTTSensorDiscovery("Error", "sensor", "mdi:alert-circle-outline", "", "", "", "diagnostic", log_error_topic, -1);
  delay(100);
	publishMQTTSensorDiscovery("Warning", "sensor", "mdi:shield-alert-outline", "", "", "", "diagnostic", log_warning_topic, -1);
  delay(100);
	publishMQTTSensorDiscovery("Info", "sensor", "mdi:information-outline", "", "", "", "diagnostic", log_info_topic, -1);
  delay(100);
	publishMQTTSensorDiscovery("IP Address", "sensor", "mdi:ip-network-outline", "", "", "", "diagnostic", ip_topic, -1);
  delay(100);
  publishMQTTSensorDiscovery("WiFi Strength", "sensor", "mdi-rss", "", "", "", "diagnostic", wifi_strength_topic, -1);
  delay(100);

  // Sensors
  publishMQTTSensorDiscovery("Water Level (US)", "sensor", "mdi-car-coolant-level", "cm", "distance", "measurement", "", USsensor_topic, 1);
  delay(100);
  publishMQTTSensorDiscovery("Water Level (IR)", "sensor", "mdi-car-coolant-level", "cm", "distance", "measurement", "", IRsensor_topic, 1);
  delay(100);

  // Parameters
  publishMQTTSensorDiscovery("Refresh Rate", "sensor", "mdi:refresh", "", "", "", "diagnostic", refreshRate_topic, 0);
  delay(100);
}

// Send back received parameters to the server
void sendParameters()
{
  publishMessage(refreshRate_topic, refreshRate, true);
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

    // Connect to WiFi
    wifiStatus = connectWifi();

    // Set MQTT server and callback
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    client.setBufferSize(4096);
    client.setKeepAlive(120);
    mqttStatus = connectMQTT();

    // Announce availability
    publishMessage(availability_topic, "connected", true);

    // Query latest firmware version
    requestFirmwareVersion();
    delay(1000);

    // Request boot parameters
    requestParameters();
    delay(1000);

    // Open MQTT connection to receive parameters
    for (int i = 0; i <= 100; i++) {
      client.loop();
      delay(50);
    }

    // Verify received parameters
    sendParameters();

    // Set OTA progress callback
    Update.onProgress([](unsigned int progress, unsigned int total) 
    {
      Serial.printf("[INFO] OTA Progress: %u%%\r", (progress * 100) / total);
      Serial.println();
    });

    // Publish diagnostic data to the server
    publishMessage(firmware_topic, currentSwVersion, true);
    publishMessage(log_info_topic, "Starting main loop", false);
    publishMessage(ip_topic, configurationUrl, true);

    // Send data to Thingboards server
    Serial.println("-----------------------------------------");
    Serial.println("Sending data to Home Assistant");
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
    Serial.println("Developer mode started");
  }
}

void loop() {
  //Firmware update mode 
  client.loop();
  delay(200);
}

/************************** Device Functions ***********************************/

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
  
  configurationUrl = WiFi.localIP().toString();

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  return true;
}

void restartESP()
{
  consoleLog("Rebooting ESP32 device", 1);
  delay(5000);
  esp_restart();
}

// Handle commands received via MQTT
void handleCommands(String message) 
{ 
  if (message == "reboot") 
  {
    consoleLog("Command received: Reboot", 1);
    restartESP();
  }
   else if (message == "request parameters") 
  {
    consoleLog("Command received: Parameter Request", 1);
    requestParameters();
  }
  else if (message == "send parameters") 
  {
    consoleLog("Command received: Parameter Send", 1);
    sendParameters();
  }
  else if (message == "send discoveries") 
  {
    consoleLog("Command received: Discovery Send", 1);
    sendDiscoveries();
  }
  else 
  {
    consoleLog("Unknown command received", 2);
  }
}

// Handle parameters received via MQTT
void handleParameters(const String& jsonPayload) 
{
    // Create a JSON document (adjust size if needed)
    StaticJsonDocument<1024> doc;

    // Deserialize the JSON
    DeserializationError error = deserializeJson(doc, jsonPayload);
    if (error) {
        consoleLog("Parameter JSON parsing failed: ", 2);
        Serial.println(error.c_str());
        return;
    }

    // Check if all expected parameters exist before assigning values
    if (!doc.containsKey("refreshRate"))
    {
        consoleLog("Missing one or more required parameters", 2);
        return;
    }

    // Extract values
    refreshRate = doc["refreshRate"].as<int>();

    // Print extracted values
    Serial.println("  Extracted Parameters:");
    Serial.print("    refreshRate: "); Serial.println(refreshRate);

    consoleLog("Parameters received and saved", 1);
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
