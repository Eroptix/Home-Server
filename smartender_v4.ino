// ESP32 Automated Bartender control code
//
// Written by Tamas Bozso for BTM Engineering
// Copyright (c) 2021 BTM Engineering
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Libraries ***********************************/
#include <WiFi.h>
#include <ThingsBoard.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <HX711.h>
#include <AccelStepper.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Espressif_Updater.h>
#include <Arduino_MQTT_Client.h>

#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"

// Statuses for updating
Espressif_Updater updater;
constexpr char CURRENT_FIRMWARE_TITLE[] = "smartender_code";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.4.3";
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 48U;
constexpr uint16_t FIRMWARE_PACKET_SIZE = 32768U;
bool currentFWSent = false;
bool updateRequestSent = false;


// Pin layout
/*                             +-----------------------+
                               | O                   O |
                               |                       |
                          3.3V | [ ]               [ ] | GND
                            EN | [ ]  ___________  [ ] | GPIO23  LOADDOUT
               BUTTON   GPIO36 | [ ] |           | [ ] | GPIO22  LOADCLK
                LIMIT   GPIO39 | [ ] |           | [ ] | GPIO01
                  LED   GPIO34 | [ ] |           | [ ] | GPIO03
               WEIGHT   GPIO35 | [ ] |           | [ ] | GPIO21  LED
              PELTIER   GPIO32 | [ ] |           | [ ] | GND
                  DIR   GPIO33 | [ ] |           | [ ] | GPIO19  PUMP1
                 STEP   GPIO25 | [ ] |___________| [ ] | GPIO18  PUMP2
                  SLP   GPIO26 | [ ]               [ ] | GPIO05  PUMP3
                  RST   GPIO27 | [ ]               [ ] | GPIO17  PUMP4
             LOADDOUT   GPIO14 | [ ]               [ ] | GPIO16  PUMP5
              LOADCLK   GPIO12 | [ ]               [ ] | GPIO04  PUMP6
                           GND | [ ]               [ ] | GPIO00  FAN
                   EN   GPIO13 | [ ]               [ ] | GPIO02  MOTOR
                        GPIO09 | [ ]               [ ] | GPIO15  WEIGHT
                        GPIO10 | [ ]               [ ] | GPIO08
                        GPIO11 | [ ]               [ ] | GPIO07
                            5V | [ ]               [ ] | GPIO06
                               |        -------        |
                               | O      | USB |      O |
                               +-----------------------+         */
                               
const int pumpOnePin = 19;
const int pumpTwoPin = 18;
const int pumpThreePin = 5;
const int pumpFourPin = 17;
const int pumpFivePin = 16;
const int pumpSixPin = 4;
const int peltierPin = 32;
const int motorPin = 2;
const int loadDOUT = 23;
const int loadCLK = 22;
const int stepperSTEP = 25;
const int stepperDIR = 33;
const int stepperSLP = 26;
const int stepperRST = 27;
const int stepperEN = 13;
const int homingPin = 39;
const int weightSensorPin = 15;
const int ledPin = 21;
const int buttonPin = 36;
const int fanPin = 0;

// Status indicators
bool pumpStatus[] = {true, false, false, false, false, false, false};
bool statusMotor = false;
bool statusPeltier = false;
bool statusLED = false;
bool statusFan = true;

// Pump pin layout
int pumpPin[] = {99, pumpOnePin, pumpTwoPin, pumpThreePin, pumpFourPin, pumpFivePin, pumpSixPin};

// Drink layout
int waterPump = 1;
int syrup1Pump = 3;
int syrup2Pump = 4;
int vodkaPump = 99;
int buttonDrink = 21;
  
// Refresh loop parameters
int refreshRate = 5; //Measurement loop length
int connectRate = 300; //Connection check loop

unsigned long previousMillis1 = 0;
const long period1 = refreshRate * 1000;

// Push button states
int lastState = HIGH;
int currentState;
unsigned long pressedTime  = 0;
bool isPressing = false;
bool isLongDetected = false;
const int longPressTime  = 3000;

// Sleep parameters
const int sleepTime  = 300000; // 5 mins
unsigned long activityMillis  = 0;

// GPIO where the DS18B20 is connected to  
AccelStepper stepper = AccelStepper(1, stepperSTEP, stepperDIR);

// Load cell settings
HX711 scale;
long standByWeight;

//Thingsboard parameters
constexpr char THINGSBOARD_SERVER[] = "192.168.0.241";
constexpr char TOKEN[] = "BSeB2AGyAMSpsKdIYCL8";
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

// Bartender Parameters
int headMovement = -32000;
int headMaxSpeed = 5000;
int headAcceleration = 3000;
float scaleCalibrationFactor = 202;
int stirrTime = 3000;
bool headOperation = true;
bool buttonCheck = true;
int glassWeight = 240;
float dailyAmount = 0;

// Pump flow speeds [g/s]
double pump1Speed = 50;
double pump2Speed = 50;
double pump3Speed = 50;
double pump4Speed = 50;
double pump5Speed = 50;
double pump6Speed = 50;

double pumpSpeed[] = {99, pump1Speed, pump2Speed, pump3Speed, pump4Speed, pump5Speed, pump6Speed};

// Setting PWM properties
const int ledChannel = 1;
const int resolution = 8;
int dutyCycleLED = 250;
const int sleepDutyCycleLED = 10;
const int operationDutyCycleLED = 250;
int operationFreq = 5000;

RPC_Response mixDrink(const RPC_Data &data)
{
  // Sleep mode off
  sleepModeOFF();
  
  // Received settings input
  String inputValue = data.as<String>();
  Serial.print("Mixing input received: ");
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

  // Water
  if(verb == 1)
  {
    // Volume: 1dl
    if (noun == 1) 
    {    
      drinks(11);
      return RPC_Response("water1dl", true);
    }

    // Volume: 2dl
    if (noun == 2) 
    {
      drinks(12);
      return RPC_Response("water2dl", true);
    }
  }

  // Soda
  if(verb == 2)
  {
    // Type1
    if (noun == 1) 
    {
      drinks(21);
      return RPC_Response("soda1", true);
    }

    // Type 2
    if (noun == 2) 
    {
      drinks(22);
      return RPC_Response("soda2", true);
    }
  }

  // Vodka Soda
  if(verb == 3)
  {
    if (noun == 1) 
    {
      drinks(31);
      return RPC_Response("soda1dl", true);
    }
  }
}

RPC_Response getLEDBrightness(const RPC_Data &data)
{
  Serial.println("----------------------------------------------------");
  Serial.println("Received the get value method for getLEDBrightness");
  Serial.print("  Sending LED brightness: ");
  Serial.print(dutyCycleLED);
  Serial.println(" %");
  Serial.println("----------------------------------------------------");
  return RPC_Response(NULL, dutyCycleLED);
}

RPC_Response setLEDBrightness(const RPC_Data &data)
{
  Serial.println("----------------------------------------------------");
  Serial.println("Received the set value method for setLEDBrightness");

  // Process data
  dutyCycleLED = data;

  Serial.print("  Set new LED duty cycle: ");
  Serial.print(dutyCycleLED);
  Serial.println(" %");
  Serial.println("----------------------------------------------------");

  return RPC_Response(NULL, dutyCycleLED);
}

RPC_Response settings(const RPC_Data &data)
{
  // Sleep mode off
  sleepModeOFF();
  
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

  // Swtich relays
  if(verb == 1)
  {
    // Switch Pumps
    if (noun == 1 && action > 0 && action < 7) 
    {
      pump(action,!pumpStatus[action]);
      return RPC_Response("pump", pumpStatus[action]);
    }

    // Switch Motor
    if (noun == 7) 
    {
      motor(!statusMotor);
      return RPC_Response("motor", statusMotor);
    }

    // Switch Peltier
    if (noun == 8) 
    {
      peltier(!statusPeltier);
      return RPC_Response("peltier", statusPeltier);
    }

    // Switch LED
    if (noun == 9) 
    {
      return RPC_Response("led", statusLED);
    }

    // Switch Fan
    if (noun == 10) 
    {
      fan(!statusFan);
      return RPC_Response("fan", statusFan);
    }
  }

   // Actions
  if(verb == 2)
  {
    // Home Z-axis
    if (noun == 1) 
    {
      stepperHoming();
      return RPC_Response("homing", true);
    }

    // Lower Z-axis
    if (noun == 2) 
    {
      stepper.runToNewPosition(headMovement);
      return RPC_Response("lowering", true);
    }

    // Raise Z-axis
    if (noun == 3) 
    {
      stepper.runToNewPosition(0);
      return RPC_Response("raising", true);
    }

    // Move Z-axis
    if (noun == 4) 
    {
      stepper.runToNewPosition(-action);
      return RPC_Response("move", true);
    }

    // Stepper sleep mode
    if (noun == 5) 
    {
      digitalWrite(stepperSLP,LOW);
      return RPC_Response("sleep", true);
    }

    // Zero Scale
    if (noun == 10) 
    {
      scale.tare();
      return RPC_Response("zeroscale", true);
    }

    // Calibration Offset
    if (noun == 11) 
    {
      scale.set_scale(action);
      return RPC_Response("scaleCalib", true);
    }

    // Calibrate scale 
    if (noun == 12) 
    {
      calibrateScale(glassWeight);
      return RPC_Response("scaleCalib", true);
    }
    
    // Flush Pipes
    if (noun == 20) 
    {
      return RPC_Response("flush", true);
    }

    // LED flash
    if (noun == 30) 
    {
      return RPC_Response("flash", true);
    }

    // Pump flow test
    if (noun == 40 && action > 0 && action < 7) 
    {
      switch (action) 
      {
        case 1:
          tb.sendAttributeData("pump1Speed", flowMeasure(action));
          break;
        case 2:
          tb.sendAttributeData("pump2Speed", flowMeasure(action));
          break;
        case 3:
          tb.sendAttributeData("pump3Speed", flowMeasure(action));
          break;
        case 4:
          tb.sendAttributeData("pump4Speed", flowMeasure(action));
          break;
        case 5:
          tb.sendAttributeData("pump5Speed", flowMeasure(action));
          break;
        case 6:
          tb.sendAttributeData("pump6Speed", flowMeasure(action));
          break;
        default:
          // statements
          break;
      }
    }

    // Recalibrate
    if (noun == 50) 
    {
      reCalibrateScale(glassWeight);
    }
  }
}

// Processes function for RPC call "settings"
RPC_Response bootSettings(const RPC_Data &data) {
     
    Serial.println("-----------------------------------------");
    Serial.println("Attributes for next cycle:");
  
    //Input data write
    Serial.print("  Glass Weight: ");
    Serial.println(data["shared"]["glassWeight"].as<int>());
    glassWeight = data["shared"]["glassWeight"].as<int>();

    Serial.print("  Button Drink: ");
    Serial.println(data["shared"]["buttonDrink"].as<int>());
    buttonDrink = data["shared"]["buttonDrink"].as<int>();

    return RPC_Response("check", 1);
}

// RPC handlers
const std::array<RPC_Callback, 5U> callbacks = {
  RPC_Callback{ "getLEDBrightness",         getLEDBrightness },
  RPC_Callback{ "setLEDBrightness",         setLEDBrightness },
  RPC_Callback{ "settings",                 settings },
  RPC_Callback{ "bootSettings",             bootSettings },
  RPC_Callback{ "mixDrink",                 mixDrink }
};

// Statuses for subscribing to shared attributes
bool subscribed = false;

// Statuses for requesting of attributes
bool requestedClient = false;
bool requestedShared = false;
bool requestedTimeProgram = false;

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
    tb.sendAttributeData("console","Firmware updated, restarting");
    Serial.println("Done, Reboot now");
    esp_restart();
    return;
  }
  Serial.println("Downloading firmware failed");
}

/// Progress callback that will be called every time we downloaded a new chunk successfully
void progressCallback(const size_t& currentChunk, const size_t& totalChuncks) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(currentChunk * 100U) / totalChuncks);
}

/************************** HomeAssistant Settings ***********************************/

// MQTT broker details
const char* mqtt_server = "192.168.0.241";
const int mqtt_port = 1783;
const char* mqtt_client_id = "ESP32_Smartender"; // Unique ID for this device

// MQTT topics
const char* drink_topic = "home/smartender/drink";
const char* weight_topic = "home/smartender/weight";
const char* calibration_topic = "home/smartender/calib";
const char* amount_topic = "home/smartender/amount";
const char* availability_topic = "home/smartender/available";
const char* restart_topic = "home/smartender/restart";

// Initalize the Mqtt client instance
WiFiClient espClient;
PubSubClient client(espClient);

bool connectMQTT()
{
  bool connection = false;
  int attempts = 1;
  
  Serial.println("Connecting to MQTT server:");
  
  while (!client.connected() && attempts < 3) 
  {
    if ( client.connect(mqtt_client_id) ) {

      // Announce availability
      client.publish(availability_topic, "online", true);

      // Subscribe to command topics
      subscribeTopic(drink_topic);

      // Send discovery payload
      //sendDiscoveries();

      Serial.println("	[DONE]");
      connection = true;
    } 
    else {
      Serial.println("	[ERROR] Failed to connected to TB server (No: " + String(attempts) + ")");
      attempts = attempts + 1;
	  
      delay(1000);
    }
  }

  return connection;
}

// MQTT Callback Function
void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
  // Sleep mode off
  sleepModeOFF();
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Received message on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  // Handle commands
  if (String(topic) == drink_topic) {
    drinks(message.toInt());
  }
  if (String(topic) == restart_topic) {
    restartESP();
  }
}

void subscribeTopic(const char* topic)
{
    client.subscribe(topic);
    Serial.print("	Subscribed to: ");
    Serial.println(topic);
}

// Publish an MQTT message with an string payload
void publishMessage(const char* topic, int payload) 
{
  char message[16]; // Buffer to store the stringified payload
  itoa(payload, message, 10); // Convert int to string
  if (client.publish(topic, message)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a double payload
void publishMessage(const char* topic, double payload) 
{
  char message[16]; // Buffer to store the stringified payload
  dtostrf(payload, 1, 2, message); // Convert double to string with 2 decimal places
  if (client.publish(topic, message)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a boolean payload
void publishMessage(const char* topic, bool payload) 
{
  const char* message = payload ? "true" : "false"; // Convert bool to "true"/"false"
  if (client.publish(topic, message)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a boolean payload
void publishMessage(const char* topic, String payload) 
{
  const char* message = payload.c_str();
  if (client.publish(topic, message)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

void setup(void)
{
  Serial.begin(115200);

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector 

  // Setup pins for MOSFET
  digitalWrite(motorPin,LOW);
  digitalWrite(pumpOnePin,LOW);
  digitalWrite(pumpTwoPin,LOW);
  digitalWrite(pumpThreePin,LOW);
  digitalWrite(pumpFourPin,LOW);
  digitalWrite(pumpFivePin,LOW);
  digitalWrite(pumpSixPin,LOW);
  digitalWrite(peltierPin,LOW);
  digitalWrite(weightSensorPin,HIGH);
  digitalWrite(stepperSLP,HIGH);
  digitalWrite(stepperRST,HIGH);
  digitalWrite(stepperEN,LOW);
  digitalWrite(fanPin,HIGH);
  
  pinMode (pumpOnePin, OUTPUT);
  pinMode (pumpTwoPin, OUTPUT);
  pinMode (pumpThreePin, OUTPUT);
  pinMode (pumpFourPin, OUTPUT);
  pinMode (pumpFivePin, OUTPUT);
  pinMode (pumpSixPin, OUTPUT);
  pinMode (motorPin, OUTPUT);
  pinMode (peltierPin, OUTPUT);
  pinMode (ledPin, OUTPUT);
  pinMode (stepperSTEP, OUTPUT);
  pinMode (stepperDIR, OUTPUT);
  pinMode (stepperSLP, OUTPUT);
  pinMode (stepperRST, OUTPUT);
  pinMode (stepperEN, OUTPUT);
  pinMode (weightSensorPin, OUTPUT);
  pinMode (fanPin, OUTPUT);
  pinMode (homingPin, INPUT_PULLUP);
  pinMode (buttonPin, INPUT_PULLUP);

  delay(1000);

  // Start cooling fan
  fan(true);

  // Initialize load cell
  scale.begin(loadDOUT, loadCLK);
  scale.set_scale(scaleCalibrationFactor);
  scale.tare();

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(headMaxSpeed);
  stepper.setAcceleration(headAcceleration);

  // Connect to WiFi
  wifiStatus = connectWifi();

  // Connect to Thingsboard Server
  tbStatus = connectTB();

  // Connect to MQTT broker
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  //Request client side attributes
  consoleLog("Requesting server attributes");
  
  constexpr std::array<const char*, 1U> REQUESTED_CLIENT_ATTRIBUTES = { "console" };
  requestedClient = requestClientAttributes(REQUESTED_CLIENT_ATTRIBUTES);

  //Request shared side attributes
  constexpr std::array<const char*, 3U> REQUESTED_BOOT_PARAMETERS = { "glassWeight",
                                                                      "buttonCheck", 
                                                                        "buttonDrink"};
  requestedShared = requestSharedAttributes(REQUESTED_BOOT_PARAMETERS);

  // Open Thingsboard connection to receive attributes
  consoleLog("Opening connection to Thingsboard server");
  for (int i = 0; i <= 100; i++) {
    tb.loop();
    delay(50);
  }
  consoleLog("Closing connection to Thingsboard server");

  // Initialize dashboard
  consoleLog("Initializing dashboard");
  tb.sendAttributeData("pump1", pumpStatus[1]);
  tb.sendAttributeData("pump2", pumpStatus[2]);
  tb.sendAttributeData("pump3", pumpStatus[3]);
  tb.sendAttributeData("pump4", pumpStatus[4]);
  tb.sendAttributeData("pump5", pumpStatus[5]);
  tb.sendAttributeData("pump6", pumpStatus[6]);
  tb.sendAttributeData("motor", statusMotor);
  tb.sendAttributeData("peltier", statusPeltier);
  tb.sendAttributeData("fan", statusFan);
  tb.sendAttributeData("dailyAmount",dailyAmount);
  
  delay(2000);

  // attach the channel to the GPIO to be controlled
  ledcSetup(ledChannel, operationFreq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, dutyCycleLED);

  // Calibrate scale
  calibrateScale(glassWeight);

  // Home Z axis
  stepperHoming();

  // Start activity timer
  activityMillis = millis();
}

void loop(void)
{
  // Store the current computer time
  unsigned long currentMillis = millis();

  // LED control
  ledcWrite(ledChannel, dutyCycleLED);
  
  // Thingsboard dashboard refresh
  tb.loop();

  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // Check button state
  if(buttonCheck) {checkPushButton();}

  if (!currentFWSent) {
    // Firmware state send at the start of the firmware
    currentFWSent = tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION) && tb.Firmware_Send_State(FW_STATE_UPDATED);
  }

  if (!updateRequestSent) {
    Serial.println("Checking for firmware update");
    const OTA_Update_Callback callback(&progressCallback, &updatedCallback, CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION, &updater, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
    updateRequestSent = tb.Start_Firmware_Update(callback);
  }

  // Add delay to the loop
  delay(200);

  // Standby loop
  if (currentMillis - previousMillis1 >= period1) 
  { 
      previousMillis1 = currentMillis;
      
      standByWeight = readWeightSensor();

      if(standByWeight<100)
      {
        consoleLog("Place your glass on the stand");
      }
      else 
      {
        consoleLog("Ready to serve drinks");
      }
      
      // Check sleep time
      if(currentMillis - activityMillis > sleepTime)
      {
        // Sleep mode activated
        sleepModeON();
      }
      
      // Check Thingsboard connection
      Serial.print("Thingsboard connection: ");
      Serial.println(tb.connected());
      if(!tb.connected())
      {
        connectTB();
      }

      // Refresh dashboard
      publishMessage(calibration_topic, scaleCalibrationFactor);
      publishMessage(amount_topic, dailyAmount);
  }
  
}

void consoleLog(const char* consoleText)
{
    //tb.sendTelemetryData("consoleLog", consoleText);
    Serial.println(consoleText);
}

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

long readWeightSensor()
{
   long weight;

   weight = scale.get_units(7);
   if (weight < 0)
   {
     weight = 0.00;
   }

   // If using double load cells
   weight = weight * 2;

  //Serial.print("Load cell: ");
  //Serial.print(weight);
  //Serial.println("g");

  tb.sendTelemetryData("weight", weight);
  publishMessage(weight_topic, static_cast<double>(weight));

  return weight;
}

void measureDrink(float drinkVolume, int scaleCalibrationOffset)
{
    long currentWeight = 0;
    int measureInterval = 100;
    int maxMeasureTime = 30000;
    
    // Zero out scale
    scale.tare();

    // Run until selected volume
    int i = 0;
    while(currentWeight < drinkVolume - scaleCalibrationOffset)
    {
      i = i + measureInterval;
      delay(measureInterval);
      currentWeight = readWeightSensor();

      // Check for error
      if(i > maxMeasureTime)
      {
        consoleLog("[ERROR] Maximum measurement time reached");
        break;
      }
    }
}

void waitForRemove()
{
  consoleLog("Remove drink from table");
  
  // Wait for drink removal
  while(readWeightSensor()> 2)
  {
    delay(1000);  
  }
  
  delay(1000);

  // Reset scale
  scale.tare();
}

void stepperHoming()
{
  Serial.println("Starting homing function");
  consoleLog("Homing stepper Z-axis");

  delay(1000);
  
  // Set the maximum speed and acceleration for homing
  stepper.setMaxSpeed(3000.0);
  stepper.setAcceleration(1000.0);

  // Move up until homing switch pressed
  while(digitalRead(homingPin)  == HIGH)
  {
    stepper.runToNewPosition(stepper.currentPosition() + 200); //Move up
  }

  // Move down until homing switch released
  while(digitalRead(homingPin)  == LOW)
  {
    stepper.runToNewPosition(stepper.currentPosition() - 100); //Move down
  }

  Serial.println("Homing finished");
  consoleLog("Stepper homing finished");
  stepper.setCurrentPosition(0);

  // Set the maximum speed and acceleration for operation
  stepper.setMaxSpeed(headMaxSpeed);
  stepper.setAcceleration(headAcceleration);
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

void motorStirring (int onTime)
{
  motor(true);
  delay(onTime);
  motor(false);  
}

void pump(int pumpID, bool state)
{  
    pumpStatus[pumpID] = state;
    
    if(state)
    {
      Serial.print("Pump [");
      Serial.print(pumpID);
      Serial.println("] : START");
      digitalWrite(pumpPin[pumpID], HIGH); 
    }
    else 
    {
      Serial.print("Pump [");
      Serial.print(pumpID);
      Serial.println("] : STOP");
      digitalWrite(pumpPin[pumpID], LOW); 
    }

     switch (pumpID) 
     {
       case 1:
         tb.sendAttributeData("pump1", pumpStatus[pumpID]);
         break;
       case 2:
         tb.sendAttributeData("pump2", pumpStatus[pumpID]);;
         break;
       case 3:
         tb.sendAttributeData("pump3", pumpStatus[pumpID]);
         break;
       case 4:
         tb.sendAttributeData("pump4", pumpStatus[pumpID]);
         break;
       case 5:
         tb.sendAttributeData("pump5", pumpStatus[pumpID]);
         break;
       case 6:
         tb.sendAttributeData("pump6", pumpStatus[pumpID]);
         break;
       default:
         // statements
         break;
     }
}

void motor(bool state)
{
    statusMotor = state;
    
    if(state)
    {
      Serial.println("Motor: START");
      digitalWrite(motorPin, HIGH); 
    }
    else 
    {
      Serial.println("Motor: STOP");
      digitalWrite(motorPin, LOW);
    }

    tb.sendAttributeData("motor", statusMotor); 
}

void peltier(bool state)
{
    statusPeltier = state;
    
    if(state)
    {
      Serial.println("Peltier: START");
      digitalWrite(peltierPin, HIGH); 
    }
    else 
    {
      Serial.println("Peltier: STOP");
      digitalWrite(peltierPin, LOW);
    }

    tb.sendAttributeData("peltier", statusPeltier); 
}

void fan(bool state)
{
    statusFan = state;
    
    if(state)
    {
      Serial.println("Fan: START");
      digitalWrite(fanPin, HIGH); 
    }
    else 
    {
      Serial.println("Fan: STOP");
      digitalWrite(fanPin, LOW);
    }

    tb.sendAttributeData("fan", statusFan); 
}

void blinkLED(int blinkDelay, int blinkNumber)
{
  for (int i = 0; i <= blinkNumber; i++) {
    ledcWrite(ledChannel, dutyCycleLED);
    delay(blinkDelay);
    ledcWrite(ledChannel, 0);
    delay(blinkDelay);
  }  
}

void calibrateScale(int calibrationWeight)
{ 
  Serial.println("---------------------------------");
  Serial.println("      Starting calibration");
  Serial.print("  Calibration weight goal: ");
  Serial.println(calibrationWeight);
  Serial.println("      ");

  consoleLog("Scale calibration started");

  // Restart weight sensor
  Serial.println("  Turning off weight sensor");
  digitalWrite(weightSensorPin,LOW);
  delay(1000);
  Serial.println("  Turning on weight sensor");
  digitalWrite(weightSensorPin,HIGH);
  
  // Reset empty scale 
  scale.tare();

  Serial.println("  Waiting for glass placement");
  
  // Wait until calibrationWeight placed on the scale
  while(readWeightSensor() < 100)
  {
    ledcWrite(ledChannel, dutyCycleLED);
    delay(500);
    ledcWrite(ledChannel, 0);
    delay(500);
  }
  Serial.print("  Glass placement detected ");
  
  long currentWeight = readWeightSensor();
  int iterationNumber = 0;

  Serial.println(currentWeight); 

  // Start calibration
  while(abs(currentWeight - calibrationWeight)>2)
  {
    // Adjust scale factor
    scaleCalibrationFactor = scaleCalibrationFactor + (currentWeight - calibrationWeight)*0.1;
    
    Serial.print("  Calibration factor: ");
    Serial.print(scaleCalibrationFactor);
    
    scale.set_scale(scaleCalibrationFactor);

    delay(500);

    // Read current weight after calibration
    currentWeight = readWeightSensor();
    
    Serial.print("  Current weight: ");
    Serial.println(currentWeight);
    
    // Check if we are stuck
    iterationNumber++;
    if(iterationNumber > 200)
    {
      blinkLED(100, 10);
      break;
    }
  }
  
  // Finished calibration
  tb.sendAttributeData("calibrationFactor", scaleCalibrationFactor);
  publishMessage(calibration_topic, scaleCalibrationFactor);
  consoleLog("Scale calibration finished");

  Serial.println("      ");
  Serial.println("      Finished calibration");
  Serial.println("---------------------------------");
}

void reCalibrateScale(int calibrationWeight)
{ 
  Serial.println("---------------------------------");
  Serial.println("      Starting calibration");
  Serial.print("  Calibration weight goal: ");
  Serial.println(calibrationWeight);
  Serial.println("      ");

  consoleLog("Scale calibration started");
  
  long currentWeight = readWeightSensor();
  int iterationNumber = 0;

  Serial.println(currentWeight); 

  // Start calibration
  while(abs(currentWeight - calibrationWeight)>2)
  {
    // Adjust scale factor
    scaleCalibrationFactor = scaleCalibrationFactor + (currentWeight - calibrationWeight)*0.1;
    
    Serial.print("  Calibration factor: ");
    Serial.print(scaleCalibrationFactor);
    
    scale.set_scale(scaleCalibrationFactor);

    delay(500);

    // Read current weight after calibration
    currentWeight = readWeightSensor();
    
    Serial.print("  Current weight: ");
    Serial.println(currentWeight);
    
    // Check if we are stuck
    iterationNumber++;
    if(iterationNumber > 200)
    {
      blinkLED(100, 10);
      break;
    }
  }
  
  // Finished calibration
  tb.sendAttributeData("calibrationFactor", scaleCalibrationFactor);
  consoleLog("Scale calibration finished");

  Serial.println("      ");
  Serial.println("      Finished calibration");
  Serial.println("---------------------------------");
}

double flowMeasure(int pumpID)
{
  Serial.println("---------------------------------");
  Serial.println("      Flow measurement function");
  
  // Reset empty scale 
  scale.tare();

  Serial.println("      Starting priming selected pump");  
  // Wait until calibrationWeight placed on the scale
  pump(pumpID,true);
  while(readWeightSensor() < 20)
  {
    delay(20);
  }
  pump(pumpID,false);
  Serial.println("      Pump succesfully primed ");

  delay(1000);

  // Starting measurement
  unsigned long startMillis = millis();

  pump(pumpID,true);
  measureDrink(100,0);
  pump(pumpID,false);

  unsigned long endMillis = millis();

  // Calculate pump speed in g/msec
  double pumpSpeed = 100/(endMillis - startMillis);

  // Return pump speed
  return pumpSpeed;
}

void sleepModeON()
{
    Serial.println("--------------------------------");
    Serial.println("        Sleep Mode ON");
    Serial.println("--------------------------------");

    consoleLog("Entering sleep mode");
    
    // Set sleep brightness
    while(dutyCycleLED > sleepDutyCycleLED)
    {
        dutyCycleLED--;
        ledcWrite(ledChannel, dutyCycleLED);
        delay(100);
    }

    // Activate sleep mode on stepper driver
    digitalWrite(stepperSLP,LOW);

    // Turn off cooling fan
    fan(false);
}

void sleepModeOFF()
{
    Serial.println("--------------------------------");
    Serial.println("        Sleep Mode OFF");
    Serial.println("--------------------------------");

    consoleLog("Waking up from sleep mode");
    
    // Reset activity timer
    activityMillis = millis();
    
    // Set normal mode brightness
    while(dutyCycleLED < operationDutyCycleLED)
    {
        dutyCycleLED++;
        ledcWrite(ledChannel, dutyCycleLED);
        delay(10);
    }

    // Turn on stepper driver
    digitalWrite(stepperSLP,HIGH);

    // Turn on cooling fan
    fan(true);
}

void checkPushButton() 
{
    static unsigned long pressedTime = 0; // Time when the button was pressed
    static bool isLongDetected = false;  // Tracks if a long press was detected
    static int lastState = HIGH;         // Last button state
    const unsigned long debounceDelay = 50; // Debounce delay in ms

    int currentState = digitalRead(buttonPin);

    if (lastState == HIGH && currentState == LOW) { // Button pressed
        unsigned long now = millis();
        if (now - pressedTime > debounceDelay) { // Debounce check
            pressedTime = now;
            isLongDetected = false;
        }
    } 
    else if (lastState == LOW && currentState == HIGH) { // Button released
        unsigned long pressDuration = millis() - pressedTime;

        if (pressDuration > longPressTime && !isLongDetected) {
            Serial.println("  --- Button press detected ---");
            
            // Sleep mode off
            sleepModeOFF();
            
            drinks(buttonDrink);
            
            isLongDetected = true;
        }
    }

    // Update last state
    lastState = currentState;
}

void dispense(int drink, float amount)
{
     // Update daily drink amount
     dailyAmount = dailyAmount + amount;
     tb.sendAttributeData("dailyAmount",dailyAmount);
     publishMessage(amount_topic, dailyAmount);

     switch (drink) 
     {
       case 1:
         pump(waterPump,true);
         measureDrink(amount,20);
         pump(waterPump,false);
         break;
       case 2:
         pump(syrup1Pump,true);
         measureDrink(amount,0);
         pump(syrup1Pump,false);
         break;
       case 3:
         pump(syrup2Pump,true);
         measureDrink(amount,0);
         pump(syrup2Pump,false);
         break;
       default:
         // statements
         Serial.println("   Unknown dispense command!");
         break;
     }
}

void drinks(int drinkID)
{
     // Drink menu
     //  - 1: Water
     //  - 2: Syrup1
     //  - 3: Syrup2
     //  - 4: Vodka

     consoleLog("Drink order received");

     switch (drinkID) 
     {
       case 11:
            Serial.println("  |MIXING: Water 1dl|  ");
            dispense(1,100);
            
            waitForRemove();
         break;
       case 12:
            Serial.println("  |MIXING: Water 2dl|  ");
            dispense(1,200);
            
            waitForRemove();
         break;
       case 21:
            Serial.println("  |MIXING: Syrup1 2dl|  ");
            if(headOperation){stepper.runToNewPosition(headMovement);}
            
            dispense(2,15); 
            
            delay(1000);   
             
            dispense(1,175);
            
            motorStirring(stirrTime);
            
            if(headOperation){stepper.runToNewPosition(0);}
            
            waitForRemove();
         break;
       case 22:
            Serial.println("  |MIXING: Syrup2 2dl|  ");
            if(headOperation){stepper.runToNewPosition(headMovement);}
            
            dispense(3,5); 
            
            delay(1000);   
             
            dispense(1,180);
            
            motorStirring(stirrTime);
            
            if(headOperation){stepper.runToNewPosition(0);}
            
            waitForRemove();
         break;
       case 31:
            Serial.println("  |MIXING: Vodka Soda|  ");
            if(headOperation){stepper.runToNewPosition(headMovement);}
            
            dispense(4,20);
            
            delay(1000);
            
            dispense(2,10);
            
            delay(1000);
            
            dispense(1,130);
            
            motorStirring(stirrTime);
            
            if(headOperation){stepper.runToNewPosition(0);}
            
            waitForRemove();
         break;
       default:
         // statements
         break;
     }
}

void restartESP()
{
  Serial.println("Rebooting ESP32");
  delay(2000);
  esp_restart();
}

void setupBootParameters(String parameter, double value){

  if (parameter == "glassWeight") {
      glassWeight = value;
      Serial.print("    [SET] glassWeight");
      Serial.print(" [TO] ");
      Serial.println(value);

  } else if (parameter == "buttonDrink") {
      buttonDrink = value;
      Serial.print("    [SET] buttonDrink");
      Serial.print(" [TO] ");
      Serial.println(value);

  } else if (parameter == "buttonCheck") {
      buttonCheck = value;
      Serial.print("    [SET] buttonCheck");
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
