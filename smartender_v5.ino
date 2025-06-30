// ESP32 Automated Bartender control code
//
// Written by Tamas Bozso for BTM Engineering
// Copyright (c) 2021 BTM Engineering
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Libraries ***********************************/
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <HX711.h>
#include <AccelStepper.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Espressif_Updater.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <RunningAverage.h>

#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "smartender";
const char* currentSwVersion = "1.2.0";
const char* deviceModel = "ESP32-NodeMCU";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";
String  firmwareUrl;
String  latestSwVersion;
bool debugMode = true;

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
  
// Refresh loop parameters
int refreshRate = 5;                                // Measurement loop length
int refreshLoop = 1;                                // Number of refresh loops
double upTime = 0;
unsigned long previousMillis1 = 0;
const long period1 = refreshRate * 1000;
unsigned long previousMillisMQTT = 0;               // MQTT reconnect timing
unsigned long previousMillisWiFi = 0;               // WiFi reconnect timing
unsigned long mqttReconnectInterval = 5000;         // Check MQTT every 5 seconds
unsigned long wifiReconnectInterval = 5000;         // Check WiFi every 5 seconds 
unsigned long wifiRetryMaxInterval = 30000;         // 30 seconds max
unsigned long mqttRetryMaxInterval = 60000;         // 60 seconds max 
bool wifiStatus = false;                            // WiFi status indicator
long wifiStrength;                                  // WiFi strength value 

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
bool sleepMode = false;

// GPIO where the DS18B20 is connected to  
AccelStepper stepper = AccelStepper(1, stepperSTEP, stepperDIR);

// Load cell settings
HX711 scale;
long standByWeight;

// Bartender Parameters
int headMovement = -32000;
int headMaxSpeed = 5000;
int headAcceleration = 3000;
float scaleCalibrationFactor = 202;
int glassWeight = 240;
float dailyAmount = 0;
bool readyToServe = false;

// Setting PWM properties
const int ledChannel = 1;
const int resolution = 8;
int dutyCycleLED = 250;
const int sleepDutyCycleLED = 10;
const int operationDutyCycleLED = 250;
int operationFreq = 5000;

// Running average filter
int MOVING_AVG_SIZE = 3;
RunningAverage weightFilter(MOVING_AVG_SIZE);

/************************** HomeAssistant Settings ***********************************/

// MQTT broker details
const char* mqtt_server = "192.168.0.241";
const int mqtt_port = 1783;
const char* mqtt_client_id = deviceName;
bool mqttStatus = false;

// MQTT topics
// Diagnostic
String ota_query_topic =                    String("home/ota/query");
String parameter_request_topic =            String("home/") + deviceName + String("/parameters/request");
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
String weight_topic =                       String("home/") + deviceName + String("/weight");
String dailyamount_topic =                  String("home/") + deviceName + String("/dailyamount");

// Switches
String pump1_state_topic =                  String("home/") + deviceName + String("/pumps/pump1/state");
String pump1_command_topic =                String("home/") + deviceName + String("/pumps/pump1/command");
String pump2_state_topic =                  String("home/") + deviceName + String("/pumps/pump2/state");
String pump2_command_topic =                String("home/") + deviceName + String("/pumps/pump2/command");
String pump3_state_topic =                  String("home/") + deviceName + String("/pumps/pump3/state");
String pump3_command_topic =                String("home/") + deviceName + String("/pumps/pump3/command");
String pump4_state_topic =                  String("home/") + deviceName + String("/pumps/pump4/state");
String pump4_command_topic =                String("home/") + deviceName + String("/pumps/pump4/command");
String pump5_state_topic =                  String("home/") + deviceName + String("/pumps/pump5/state");
String pump5_command_topic =                String("home/") + deviceName + String("/pumps/pump5/command");
String pump6_state_topic =                  String("home/") + deviceName + String("/pumps/pump6/state");
String pump6_command_topic =                String("home/") + deviceName + String("/pumps/pump6/command");
String motor_state_topic =                  String("home/") + deviceName + String("/motor/state");
String motor_command_topic =                String("home/") + deviceName + String("/motor/command");
String fan_state_topic =                    String("home/") + deviceName + String("/fan/state");
String fan_command_topic =                  String("home/") + deviceName + String("/fan/command");
String peltier_state_topic =                String("home/") + deviceName + String("/peltier/state");
String peltier_command_topic =              String("home/") + deviceName + String("/peltier/command");

// Parameters
String button_topic =                       String("home/") + deviceName + String("/button");
String drink_topic =                        String("home/") + deviceName + String("/drink");
String calibration_topic =                  String("home/") + deviceName + String("/calibration");

// Numbers
String glass_weight_state_topic =           String("home/") + deviceName + String("/parameters/glassWeight/state");
String glass_weight_command_topic =         String("home/") + deviceName + String("/parameters/glassWeight/command");


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
      subscribeTopic(drink_topic);
      subscribeTopic(pump1_command_topic);
      subscribeTopic(pump2_command_topic);
      subscribeTopic(pump3_command_topic);
      subscribeTopic(pump4_command_topic);
      subscribeTopic(pump5_command_topic);
      subscribeTopic(pump6_command_topic);
      subscribeTopic(motor_command_topic);
      subscribeTopic(peltier_command_topic);
      subscribeTopic(fan_command_topic);
      subscribeTopic(glass_weight_command_topic);

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
  else if (String(topic) == command_topic) 
  {
    handleCommands(message);
  }
  else if (String(topic) == drink_topic) 
  {
    handleDrinks(message);
  }
  else if (String(topic) == pump1_command_topic) 
  {
    handlePumps(message, 1);
  }
  else if (String(topic) == pump2_command_topic) 
  {
    handlePumps(message, 2);
  }
  else if (String(topic) == pump3_command_topic) 
  {
    handlePumps(message, 3);
  }
  else if (String(topic) == pump4_command_topic) 
  {
    handlePumps(message, 4);
  }
  else if (String(topic) == pump5_command_topic) 
  {
    handlePumps(message, 5);
  }
  else if (String(topic) == pump6_command_topic) 
  {
    handlePumps(message, 6);
  }
  else if (String(topic) == motor_command_topic) 
  {
    handleMotor(message);
  }
  else if (String(topic) == fan_command_topic) 
  {
    handleFan(message);
  }
  else if (String(topic) == peltier_command_topic) 
  {
    handlePeltier(message);
  }
  else if (String(topic) == glass_weight_command_topic) 
  {
    handleParameter("glassWeight",message);
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
  publishMessage(parameter_request_topic, "request", false);
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
void publishMQTTSensorDiscovery(String name, String stateTopic, String icon = "", String unitOfMeasurement = "", String deviceClass = "", String stateClass = "", String entityCategory = "", int displayPrecision = -1) 
{
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/sensor/" + uniqueID + "/config";

  DynamicJsonDocument doc(1024);
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["state_topic"] = stateTopic;

  if (!icon.isEmpty()) doc["icon"] = icon;
  if (!unitOfMeasurement.isEmpty()) doc["unit_of_meas"] = unitOfMeasurement;
  if (!deviceClass.isEmpty()) doc["device_class"] = deviceClass;
  if (!stateClass.isEmpty()) doc["state_class"] = stateClass;
  if (!entityCategory.isEmpty()) doc["entity_category"] = entityCategory;
  if (displayPrecision != -1) doc["suggested_display_precision"] = displayPrecision;

  doc["availability_topic"] = availability_topic;
  doc["payload_available"] = "connected";
  doc["payload_not_available"] = "connection lost";

  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceName);
  device["name"] = deviceName;
  device["model"] = deviceModel;
  device["manufacturer"] = deviceManufacturer;

  String payload;
  serializeJson(doc, payload);
  publishMessage(topic.c_str(), payload.c_str(), true);
}

// Create binary sensor discovery payload
void publishMQTTBinarySensorDiscovery(String name, String stateTopic, String icon = "", String deviceClass = "", String entityCategory = "") 
{
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/binary_sensor/" + uniqueID + "/config";

  DynamicJsonDocument doc(1024);
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["state_topic"] = stateTopic;
  doc["payload_on"] = "ON";
  doc["payload_off"] = "OFF";

  if (!icon.isEmpty()) doc["icon"] = icon;
  if (!deviceClass.isEmpty()) doc["device_class"] = deviceClass;
  if (!entityCategory.isEmpty()) doc["entity_category"] = entityCategory;

  doc["availability_topic"] = availability_topic;
  doc["payload_available"] = "connected";
  doc["payload_not_available"] = "connection lost";

  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceName);
  device["name"] = deviceName;
  device["model"] = deviceModel;
  device["manufacturer"] = deviceManufacturer;

  String payload;
  serializeJson(doc, payload);
  publishMessage(topic.c_str(), payload.c_str(), true);
}

// Create switch discovery payload
void publishMQTTSwitchDiscovery(String name, String commandTopic, String stateTopic, String icon = "") 
{
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/switch/" + uniqueID + "/config";

  DynamicJsonDocument doc(1024);
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["command_topic"] = commandTopic;
  doc["state_topic"] = stateTopic;
  doc["payload_on"] = "manual on";
  doc["payload_off"] = "manual off";
  doc["state_on"] = "ON";
  doc["state_off"] = "OFF";
  doc["retain"] = true;

  if (!icon.isEmpty()) doc["icon"] = icon;

  doc["availability_topic"] = availability_topic;
  doc["payload_available"] = "connected";
  doc["payload_not_available"] = "connection lost";

  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceName);
  device["name"] = deviceName;
  device["model"] = deviceModel;
  device["manufacturer"] = deviceManufacturer;

  String payload;
  serializeJson(doc, payload);
  publishMessage(topic.c_str(), payload.c_str(), true);
}

// Create select discovery payload
void publishMQTTSelectDiscovery(String name, String commandTopic, String stateTopic, std::vector<String> options, String icon = "") 
{
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/select/" + uniqueID + "/config";

  DynamicJsonDocument doc(1024);
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["command_topic"] = commandTopic;
  doc["state_topic"] = stateTopic;
  doc["retain"] = true;

  JsonArray opts = doc.createNestedArray("options");
  for (auto& opt : options) opts.add(opt);

  if (!icon.isEmpty()) doc["icon"] = icon;

  doc["availability_topic"] = availability_topic;
  doc["payload_available"] = "connected";
  doc["payload_not_available"] = "connection lost";

  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceName);
  device["name"] = deviceName;
  device["model"] = deviceModel;
  device["manufacturer"] = deviceManufacturer;

  String payload;
  serializeJson(doc, payload);
  publishMessage(topic.c_str(), payload.c_str(), true);
}

// Create button discovery payload
void publishMQTTButtonDiscovery(String name, String commandTopic, String icon = "", bool optimistic = false) 
{
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/button/" + uniqueID + "/config";

  DynamicJsonDocument doc(1024);
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["command_topic"] = commandTopic;

  if (optimistic) doc["optimistic"] = true;
  if (!icon.isEmpty()) doc["icon"] = icon;

  doc["availability_topic"] = availability_topic;
  doc["payload_available"] = "connected";
  doc["payload_not_available"] = "connection lost";

  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceName);
  device["name"] = deviceName;
  device["model"] = deviceModel;
  device["manufacturer"] = deviceManufacturer;

  String payload;
  serializeJson(doc, payload);
  publishMessage(topic.c_str(), payload.c_str(), true);
}

// Create number discovery payload
void publishMQTTNumberDiscovery(String name, String commandTopic, String stateTopic, float minValue, float maxValue, float step, String icon = "", String unit = "", bool optimistic = false) 
{
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/number/" + uniqueID + "/config";

  DynamicJsonDocument doc(1024);
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["command_topic"] = commandTopic;
  doc["state_topic"] = stateTopic;
  doc["min"] = minValue;
  doc["max"] = maxValue;
  doc["step"] = step;
  doc["retain"] = true;

  if (!unit.isEmpty()) doc["unit_of_meas"] = unit;
  if (!icon.isEmpty()) doc["icon"] = icon;
  if (optimistic) doc["optimistic"] = true;

  doc["availability_topic"] = availability_topic;
  doc["payload_available"] = "connected";
  doc["payload_not_available"] = "connection lost";

  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceName);
  device["name"] = deviceName;
  device["model"] = deviceModel;
  device["manufacturer"] = deviceManufacturer;

  String payload;
  serializeJson(doc, payload);
  publishMessage(topic.c_str(), payload.c_str(), true);
}

// Create climate discovery payload
void publishMQTTClimateDiscovery(String name, String climate_temperature_current_topic, String climate_temperature_state_topic, String climate_temperature_command_topic, String climate_mode_state_topic, String climate_mode_command_topic ) 
{
    // Construct IDs
    String uniqueID = "climate-" + String(deviceName);
    String objectID = "climate_" + String(deviceName);
    
    // Construct discovery topic
    String topic = "homeassistant/climate/" + uniqueID + "/config";
    
    // Create a JSON document
    DynamicJsonDocument doc(1024);  // Adjust the size if necessary

    // Fill the JSON document with values
    doc["name"] = name;
    doc["unique_id"] = uniqueID;
    doc["object_id"] = objectID;
    doc["availability_topic"] = availability_topic;
    doc["payload_available"] = "connected";
    doc["payload_not_available"] = "connection lost";
    doc["current_temperature_topic"] = climate_temperature_current_topic;
    doc["temperature_state_topic"] = climate_temperature_state_topic;
    doc["temperature_command_topic"] = climate_temperature_command_topic;
    doc["mode_state_topic"] = climate_mode_state_topic;
    doc["mode_command_topic"] = climate_mode_command_topic;
    doc["min_temp"] = 12;
    doc["max_temp"] = 30;
    doc["temp_step"] = 1.0;
    doc["precision"] = 0.1;
    doc["retain"] = true;

    JsonArray modes = doc.createNestedArray("modes");
      modes.add("off");
      modes.add("heat");
      modes.add("cool");
      modes.add("auto");

    // Add the device object
    JsonObject device = doc.createNestedObject("device");
    JsonArray identifiers = device.createNestedArray("identifiers");
    identifiers.add(deviceName);

    device["name"] = deviceName;
    device["model"] = deviceModel;
    device["manufacturer"] = deviceManufacturer;

    // Serialize the JSON document to a string
    String payload;

    if (serializeJson(doc, payload) == 0) {
        Serial.println("Failed to serialize JSON!");
        return;
    }
    
    // Publish discovery message
    publishMessage(topic.c_str(), payload.c_str(), true);
}

// Create dynamic discovery payload
void publishMQTTDiscoveryEntity(
  String name,
  String deviceType,
  String stateTopic,
  String commandTopic = "",
  String icon = "",
  String unitOfMeasurement = "",
  String deviceClass = "",
  String stateClass = "",
  String entityCategory = "",
  int displayPrecision = -1,
  std::vector<String> options = {},       // for select
  float minValue = NAN,                   // for number
  float maxValue = NAN,
  float stepValue = NAN,
  bool optimistic = false                 // for button/number
) {
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/" + deviceType + "/" + uniqueID + "/config";

  DynamicJsonDocument doc(2048);

  // Common fields
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["state_topic"] = stateTopic;
  if (!icon.isEmpty()) doc["icon"] = icon;
  if (!entityCategory.isEmpty()) doc["entity_category"] = entityCategory;
  if (!unitOfMeasurement.isEmpty()) doc["unit_of_meas"] = unitOfMeasurement;
  if (!deviceClass.isEmpty()) doc["device_class"] = deviceClass;
  if (!stateClass.isEmpty()) doc["state_class"] = stateClass;
  if (displayPrecision != -1) doc["suggested_display_precision"] = displayPrecision;

  doc["availability_topic"] = availability_topic;
  doc["payload_available"] = "connected";
  doc["payload_not_available"] = "connection lost";

  // Type-specific configuration
  if (deviceType == "switch") {
    doc["command_topic"] = commandTopic;
    doc["payload_on"] = "manual_on";
    doc["payload_off"] = "auto";
    doc["state_on"] = "manual_on";
    doc["state_off"] = "auto";
    doc["retain"] = true;
  }
  else if (deviceType == "select") {
    doc["command_topic"] = commandTopic;
    JsonArray opts = doc.createNestedArray("options");
    for (const auto& opt : options) opts.add(opt);
    doc["retain"] = true;
  }
  else if (deviceType == "binary_sensor") {
    doc["payload_on"] = "ON";
    doc["payload_off"] = "OFF";
  }
  else if (deviceType == "button") {
    doc["command_topic"] = commandTopic;
    if (optimistic) doc["optimistic"] = true;
  }
  else if (deviceType == "number") {
    doc["command_topic"] = commandTopic;
    if (!isnan(minValue)) doc["min"] = minValue;
    if (!isnan(maxValue)) doc["max"] = maxValue;
    if (!isnan(stepValue)) doc["step"] = stepValue;
    doc["retain"] = true;
    if (optimistic) doc["optimistic"] = true;
  }

  // Device block
  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceName);
  device["name"] = deviceName;
  device["model"] = deviceModel;
  device["manufacturer"] = deviceManufacturer;

  // Serialize and publish
  String payload;
  if (serializeJson(doc, payload) == 0) {
    Serial.println("Failed to serialize MQTT discovery JSON!");
    return;
  }

  publishMessage(topic.c_str(), payload.c_str(), true);
}

// Send discovery topics to Home Assistant
void sendDiscoveries()
{
  // Diagnostics
  publishMQTTSensorDiscovery("Up Time", uptime_topic,"mdi:clock", "h", "duration", "total_increasing", "diagnostic", 0);
  delay(100);
	publishMQTTSensorDiscovery("OTA Status", ota_status_topic, "mdi:update", "", "", "", "diagnostic", -1);
  delay(100);
	publishMQTTSensorDiscovery("Firmware Version", firmware_topic, "mdi:application-outline", "", "", "", "diagnostic", -1);
  delay(100);
	publishMQTTSensorDiscovery("Error", log_error_topic, "mdi:alert-circle-outline", "", "", "", "diagnostic", -1);
  delay(100);
	publishMQTTSensorDiscovery("Warning", log_warning_topic, "mdi:shield-alert-outline", "", "", "", "diagnostic", -1);
  delay(100);
	publishMQTTSensorDiscovery("Info", log_info_topic, "mdi:information-outline", "", "", "", "diagnostic", -1);
  delay(100);
	publishMQTTSensorDiscovery("IP Address", ip_topic, "mdi:ip-network-outline", "", "", "", "diagnostic", -1);
  delay(100);
  publishMQTTSensorDiscovery("WiFi Strength", wifi_strength_topic, "mdi:access-point", "", "", "", "diagnostic", -1);
  delay(100);
  publishMQTTSensorDiscovery("Calibration", calibration_topic, "mdi:calculator-variant-outline", "", "", "measurement", "diagnostic", 0);
  delay(100);

  // Sensors
  publishMQTTSensorDiscovery("Weight", weight_topic, "mdi:weight", "g", "weight", "measurement", "", 0);
  delay(100);
  publishMQTTSensorDiscovery("Daily Amount", dailyamount_topic, "mdi:cup", "L", "volume", "measurement", "", 0);
  delay(100);

  // Switches
  publishMQTTSwitchDiscovery("Pump1", pump1_command_topic, pump1_state_topic, "mdi:pump");
  delay(100);
  publishMQTTSwitchDiscovery("Pump2", pump2_command_topic, pump2_state_topic, "mdi:pump");
  delay(100);
  publishMQTTSwitchDiscovery("Pump3", pump3_command_topic, pump3_state_topic, "mdi:pump");
  delay(100);
  publishMQTTSwitchDiscovery("Pump4", pump4_command_topic, pump4_state_topic, "mdi:pump");
  delay(100);
  publishMQTTSwitchDiscovery("Pump5", pump5_command_topic, pump5_state_topic, "mdi:pump");
  delay(100);
  publishMQTTSwitchDiscovery("Pump6", pump6_command_topic, pump6_state_topic, "mdi:pump");
  delay(100);
  publishMQTTSwitchDiscovery("Motor", motor_command_topic, motor_state_topic, "mdi:rotate-360");
  delay(100);
  publishMQTTSwitchDiscovery("Fan", fan_command_topic, fan_state_topic, "mdi:fan");
  delay(100);
  publishMQTTSwitchDiscovery("peltier", peltier_command_topic, peltier_state_topic, "mdi:ice-pop");
  delay(100);

  // Numbers
  publishMQTTNumberDiscovery("Glass Weight", glass_weight_command_topic, glass_weight_state_topic, 100, 400, 1.0, "mdi:cup", "g", true);
  delay(100);
}

/************************** Setup function ***********************************/

void setup(void)
{
  Serial.begin(115200);

  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector 

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

  // Initialize load cell
  scale.begin(loadDOUT, loadCLK);
  scale.set_scale(scaleCalibrationFactor);
  scale.tare();

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(headMaxSpeed);
  stepper.setAcceleration(headAcceleration);

  // Initialize moving average filter
  weightFilter.clear();

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

  // Initialize dashboard
  consoleLog("Initializing dashboard", 1);
  publishMessage(pump1_state_topic, pumpStatus[1] ? "ON" : "OFF", false);
  publishMessage(pump2_state_topic, pumpStatus[2] ? "ON" : "OFF", false);
  publishMessage(pump3_state_topic, pumpStatus[3] ? "ON" : "OFF", false);
  publishMessage(pump4_state_topic, pumpStatus[4] ? "ON" : "OFF", false);
  publishMessage(pump5_state_topic, pumpStatus[5] ? "ON" : "OFF", false);
  publishMessage(pump6_state_topic, pumpStatus[6] ? "ON" : "OFF", false);
  publishMessage(motor_state_topic, statusMotor ? "ON" : "OFF", false);
  publishMessage(fan_state_topic, statusFan ? "ON" : "OFF", false);
  publishMessage(peltier_state_topic, statusPeltier ? "ON" : "OFF", false);
  publishMessage(dailyamount_topic, dailyAmount, false);
  
  delay(2000);

  // Start cooling fan
  fan(true);

  // attach the channel to the GPIO to be controlled
  ledcSetup(ledChannel, operationFreq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, dutyCycleLED);

  // Home Z axis
  if(!debugMode){stepperHoming();}

  // Calibrate scale
  if(!debugMode){calibrateScale(glassWeight);}

  // Start activity timer
  activityMillis = millis();
}

/************************** Main loop ***********************************/

void loop(void)
{
  // Store the current computer time
  unsigned long currentMillis = millis();

  // LED control
  ledcWrite(ledChannel, dutyCycleLED);

  // Update connection status
  wifiStatus = (WiFi.status() == WL_CONNECTED);
  mqttStatus = client.connected();

  // Frequent WiFi reconnect attempt (every 5 seconds)
  if (!wifiStatus && (currentMillis - previousMillisWiFi >= wifiReconnectInterval)) 
  {
      previousMillisWiFi = currentMillis;
      Serial.println("[WARNING] WiFi Disconnected! Attempting to reconnect...");
      
      if (connectWifi())  
      { 
          Serial.println("[WARNING] WiFi Reconnected!");
          wifiReconnectInterval = 5000;  // Reset to 5s
      }
      else
      { 
          wifiReconnectInterval = min(wifiReconnectInterval * 2, wifiRetryMaxInterval); // Exponential backoff
      }
  }

  // Frequent MQTT reconnect attempt (every 5 seconds)
  if (!mqttStatus && (currentMillis - previousMillisMQTT >= mqttReconnectInterval)) 
  {
      previousMillisMQTT = currentMillis;
      Serial.println("[WARNING] MQTT Disconnected! Attempting to reconnect...");
      
      if (connectMQTT())  
      { 
          Serial.println("[WARNING] MQTT Reconnected!");
          mqttReconnectInterval = 5000;  // Reset to 5s
      }
      else
      { 
          mqttReconnectInterval = min(mqttReconnectInterval * 2, mqttRetryMaxInterval); // Exponential backoff
      }
  }

  // Check sleep time
  if (!sleepMode && (currentMillis - activityMillis > sleepTime))
  {
    // Sleep mode activated
    sleepModeON();
  }
  
  client.loop();

  // Check button state
  checkPushButton();

  // Standby loop
  if (!sleepMode && currentMillis - previousMillis1 >= period1) 
  { 
      previousMillis1 = currentMillis;

      // Uptime calculation
      refreshLoop++;
      upTime = (refreshLoop * refreshRate) / 60; // Return uptime in minutes
      Serial.print("Loop Number: ");
      Serial.println(refreshLoop);

      // Diagnostics
      wifiStrength = WiFi.RSSI();
      
      standByWeight = readWeightSensor();

      if(standByWeight<100)
      {
        consoleLog("Place your glass on the stand", 1);
        readyToServe = false;
      }
      else 
      {
        consoleLog("Ready to serve drinks", 1);
        readyToServe = true;
      }

      // Send diagnostics to Home Assistant
      publishMessage(wifi_strength_topic, (double)wifiStrength, false);
      publishMessage(uptime_topic, upTime, false);

      // Refresh dashboard
      publishMessage(calibration_topic, (double)scaleCalibrationFactor, false);
      publishMessage(dailyamount_topic, (double)dailyAmount, false);
  }

  // Add delay to the loop
  delay(200);
}

/************************** Device Functions ***********************************/

void consoleLog(String consoleText, int logLevel)
{
    switch (logLevel) 
    {
      case 1:
        Serial.print("[INFO] ");  
        publishMessage(log_info_topic, consoleText, false);
        break;
      case 2:
        Serial.print("[WARNING] ");
        publishMessage(log_warning_topic, consoleText, false);
        break;
      case 3:
        Serial.print("[ERROR] ");
        publishMessage(log_error_topic, consoleText, false);
        break;
      default:
        consoleLog("Unknown log warning level", 2);
        break;
    }

    Serial.println(consoleText);
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

  weightFilter.addValue(weight); // Add new value to filter
  long filteredWeight = weightFilter.getAverage(); // Get smoothed value

  //Serial.print("Load cell: ");
  //Serial.print(weight);
  //Serial.println("g");

  publishMessage(weight_topic, static_cast<double>(weight), false);

  return weight;
}

void measureDrink(float drinkVolume, int scaleCalibrationOffset)
{
  if(!debugMode)
  {
    long currentWeight = 0;
    int measureInterval = 100;
    int maxMeasureTime = 30000;
    
    // Zero out scale
    scale.tare();

    unsigned long startTime = millis();
  
    // Run until selected volume
    while(currentWeight < drinkVolume - scaleCalibrationOffset)
    {
      delay(measureInterval);
      currentWeight = readWeightSensor();
  
    // Timeout check
    if (millis() - startTime > maxMeasureTime)
    {
        consoleLog("Maximum measurement time reached", 3);
        break;
    }
    }
  }
  else
  {
    // Simulated measurement
    long currentWeight = 0;
    while(currentWeight < drinkVolume - scaleCalibrationOffset)
    {
      delay(200);
      currentWeight++;
    }
  }
}

void waitForRemove()
{
  consoleLog("Remove finished drink from the table", 1);
  
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
  consoleLog("Homing stepper Z-axis", 1);

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

  consoleLog("Stepper homing finished", 1);
  stepper.setCurrentPosition(0);

  // Set the maximum speed and acceleration for operation
  stepper.setMaxSpeed(headMaxSpeed);
  stepper.setAcceleration(headAcceleration);
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
          publishMessage(pump1_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 2:
          publishMessage(pump2_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 3:
          publishMessage(pump3_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 4:
          publishMessage(pump4_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 5:
          publishMessage(pump5_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 6:
          publishMessage(pump6_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
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

    publishMessage(motor_state_topic, statusMotor ? "ON" : "OFF", false); 
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

    publishMessage(peltier_state_topic, statusPeltier ? "ON" : "OFF", false);
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

    publishMessage(fan_state_topic, statusFan ? "ON" : "OFF", false);
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

  consoleLog("Scale calibration process started", 1);

  // Restart weight sensor
  Serial.println("  Turning off weight sensor");
  digitalWrite(weightSensorPin,LOW);
  delay(1000);
  Serial.println("  Turning on weight sensor");
  digitalWrite(weightSensorPin,HIGH);
  
  // Reset empty scale 
  scale.tare();

  consoleLog("Place etalon glass for calibration", 1);
  
  // Detect glass placement with debounce
  unsigned long glassDetectedTime = 0;
  while (true) {
      if (readWeightSensor() > 100) {
          if (glassDetectedTime == 0) {
              glassDetectedTime = millis(); // Start debounce timer
          } 
          else if (millis() - glassDetectedTime > 1000) { // Confirm detection
              consoleLog("Glass placement detected for calibration", 1);
              break;
          }
      } 
      else {
          glassDetectedTime = 0; // Reset if weight is removed
      }
      
      ledcWrite(ledChannel, dutyCycleLED);
      delay(500);
      ledcWrite(ledChannel, 0);
      delay(500);
  }
  
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

    delay(300);

    // Read current weight after calibration
    currentWeight = readWeightSensor();
    
    Serial.print("  Current weight: ");
    Serial.println(currentWeight);
    
    // Check if we are stuck
    iterationNumber++;
    if(iterationNumber > 200)
    {
      blinkLED(100, 10);
      consoleLog("Failed to calibrate weight sensors", 3);
      break;
    }
  }
  
  // Finished calibration
  publishMessage(calibration_topic, scaleCalibrationFactor, false);
  consoleLog("Scale calibration finished", 1);

  Serial.println("      ");
  Serial.println("      Finished calibration");
  Serial.println("---------------------------------");
}

void adjustScaleCalibration(int calibrationWeight)
{
  long currentWeight = readWeightSensor();
  int iterationNumber = 0;

  Serial.println(currentWeight); 

  // Start calibration
  while(abs(currentWeight - calibrationWeight)>2)
  {
    // Adjust scale factor
    scaleCalibrationFactor = scaleCalibrationFactor + (currentWeight - calibrationWeight)*0.05;
    
    Serial.print("  Calibration factor: ");
    Serial.print(scaleCalibrationFactor);
    
    scale.set_scale(scaleCalibrationFactor);

    delay(300);

    // Read current weight after calibration
    currentWeight = readWeightSensor();
    
    Serial.print("  Current weight: ");
    Serial.println(currentWeight);
    
    // Check if we are stuck
    iterationNumber++;
    if(iterationNumber > 100)
    {
      blinkLED(100, 10);
      consoleLog("Failed to calibrate weight sensors", 3);
      break;
    }
  }
  
  // Finished calibration
  publishMessage(calibration_topic, scaleCalibrationFactor, false);
  consoleLog("Scale calibration finished", 1);

  Serial.println("      ");
  Serial.println("      Finished calibration");
  Serial.println("---------------------------------");
}

void sleepModeON()
{
    Serial.println("--------------------------------");
    Serial.println("        Sleep Mode ON");
    Serial.println("--------------------------------");

    consoleLog("Entering sleep mode", 1);

    sleepMode = true;
    
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
  
  // Always reset activity timer, regardless of sleep state
  activityMillis = millis();  

  if (!sleepMode) return;  // Prevent unnecessary re-execution if already awake

  consoleLog("Waking up from sleep mode", 1);

  sleepMode = false;
  
  // Set normal mode brightness
  while(dutyCycleLED < operationDutyCycleLED)
  {
      dutyCycleLED++;
      ledcWrite(ledChannel, dutyCycleLED);
      delay(10);
  }

  // Turn on stepper driver
  digitalWrite(stepperSLP, HIGH);

  // Turn on cooling fan
  fan(true);
}

void checkPushButton() 
{
    static unsigned long pressedTime = 0; 
    static bool isLongDetected = false;
    static int lastState = HIGH;
    
    const unsigned long debounceDelay = 50;  // Debounce time
    const unsigned long longPressTime = 1000; // Define long press duration

    int currentState = digitalRead(buttonPin);
    unsigned long now = millis();

    // Button press detected
    if (lastState == HIGH && currentState == LOW) {  
        if (now - pressedTime > debounceDelay) { // Debounce check
            pressedTime = now;
            isLongDetected = false;
        }
    }

    // Button held down (long press detected)
    if (currentState == LOW && (now - pressedTime >= longPressTime) && !isLongDetected) {
        Serial.println("  --- Long button press detected ---");

        // Sleep mode off
        sleepModeOFF();

        // Publish manual drink trigger
        publishMessage(button_topic, "drink", false);

        isLongDetected = true;  // Prevents multiple triggers
    }

    // Button release detection (short press)
    if (lastState == LOW && currentState == HIGH) {  
        unsigned long pressDuration = now - pressedTime;

        if (pressDuration < longPressTime && pressDuration > debounceDelay) {
            Serial.println("  --- Short button press detected ---");

            // Handle short press action (if needed)
            //publishMessage(button_topic, "short_press", false);
        }
    }

    lastState = currentState;
}

void dispense(int drink, float amount)
{
     // Update daily drink amount
     dailyAmount = dailyAmount + amount;
     publishMessage(dailyamount_topic, dailyAmount, false);

     switch (drink) 
     {
      case 1:
         pump(1,true);
         measureDrink(amount,20);
         pump(1,false);
         break;
      case 2:
         pump(2,true);
         measureDrink(amount,0);
         pump(2,false);
         break;
      case 3:
         pump(3,true);
         measureDrink(amount,0);
         pump(3,false);
         break;
      case 4:
         pump(4,true);
         measureDrink(amount,0);
         pump(4,false);
         break;
      case 5:
         pump(5,true);
         measureDrink(amount,0);
         pump(5,false);
         break;
      case 6:
         pump(6,true);
         measureDrink(amount,0);
         pump(6,false);
         break;
       default:
         // statements
         consoleLog("Unknown dispense command received", 2);
         break;
     }
}

// Handle commands received via MQTT
void handleCommands(String message) 
{
  // Turn off sleep mode
  sleepModeOFF();
  
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
  else if (message == "send discoveries") 
  {
    consoleLog("Command received: Discovery Send", 1);
    sendDiscoveries();
  }
  else if (message == "sleep on") 
  {
    consoleLog("Command received: Sleep Mode Activated", 1);
    sleepModeON();
  }
  else if (message == "adjust scale") 
  {
    consoleLog("Command received: Scale Calibration", 1);
    adjustScaleCalibration(glassWeight);
  }
  else 
  {
    consoleLog("Unknown command received", 2);
  }
}

void handleParameter(String parameterName, String value) 
{
  if (parameterName == "glassWeight") 
  {
    glassWeight = value.toInt();
  }
  else 
  {
    consoleLog("Unknown parameter: " + parameterName, 2);
    return;
  }

  consoleLog("Parameter '" + parameterName + "' set to " + value, 1);
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
    if (!doc.containsKey("glassWeight"))
    {
        consoleLog("Missing one or more required parameters", 2);
        return;
    }

    // Extract values
    glassWeight = doc["glassWeight"].as<int>();

    // Print extracted values
    Serial.println("  Extracted Parameters:");
    Serial.print("    glassWeight: "); Serial.println(glassWeight);

    consoleLog("Parameters received and saved", 1);
}

// Handle drink recipes received via MQTT
void handleDrinks(const String& jsonPayload) 
{
  // Sample JSON recipe
  /*  
  {
    "name": "Mojito",
    "pump1": 50,
    "pump2": 20,
    "pump3": 0,
    "pump4": 30,
    "pump5": 0,
    "pump6": 10,
    "headMovement": true,
    "stirring": 5
  }
  */

  // Turn off sleep mode
  sleepModeOFF();

  // Create a JSON document (adjust size if needed)
  StaticJsonDocument<1024> doc;

  // Deserialize the JSON
  DeserializationError error = deserializeJson(doc, jsonPayload);
  if (error) {
      consoleLog("Drink recipe JSON parsing failed: ", 2);
      Serial.println(error.c_str());
      return;
  }

  // Check if all expected parameters exist before assigning values
  if (!doc.containsKey("name") ||
      !doc.containsKey("pump1") ||
      !doc.containsKey("pump2") ||
      !doc.containsKey("pump3") ||
      !doc.containsKey("pump4") ||
      !doc.containsKey("pump5") ||
      !doc.containsKey("pump6") ||
      !doc.containsKey("headMovement") ||
      !doc.containsKey("stirring"))
  {
      consoleLog("Missing one or more required recipe parameters", 2);
      return;
  }

  // Extract values
  String name     = doc["name"].as<String>();
  int pump1       = doc["pump1"].as<int>();
  int pump2       = doc["pump2"].as<int>();
  int pump3       = doc["pump3"].as<int>();
  int pump4       = doc["pump4"].as<int>();
  int pump5       = doc["pump5"].as<int>();
  int pump6       = doc["pump6"].as<int>();
  bool head       = doc["headMovement"].as<bool>();
  int stirring    = doc["stirring"].as<int>();

  // Print extracted values
  Serial.println("  Extracted Recipe Parameters:");
  Serial.print("    name: ");             Serial.println(name);
  Serial.print("    pump1: ");            Serial.println(pump1);
  Serial.print("    pump2: ");            Serial.println(pump2);
  Serial.print("    pump3: ");            Serial.println(pump3);
  Serial.print("    pump4: ");            Serial.println(pump4);
  Serial.print("    pump5: ");            Serial.println(pump5);
  Serial.print("    pump6: ");            Serial.println(pump6);
  Serial.print("    headMovement: ");     Serial.println(head);
  Serial.println("    stirring: ");       Serial.println(stirring);

  // Check total volume
  int totalVolume = pump1 + pump2 + pump3 + pump4 + pump5 + pump6;
  if(totalVolume > 200)
  {
    consoleLog("Total requested drink volume exceeds maximum", 2);
    return;
  }

  if(readyToServe)
  {
    // Start dispensing drink with the received recipe
    consoleLog(String("Mixing drink: ") + name, 1);

    if(head){stepper.runToNewPosition(headMovement);}       // Lower stirring head
    if(pump3 != 0){dispense(3, pump3);}                     // Dispense with pump3
    delay(200);
    if(pump4 != 0){dispense(4, pump4);}                     // Dispense with pump4
    delay(200);
    if(pump5 != 0){dispense(5, pump5);}                     // Dispense with pump5
    delay(200);
    if(pump6 != 0){dispense(6, pump6);}                     // Dispense with pump6
    delay(200);
    if(pump1 != 0){dispense(1, pump1);}                     // Dispense with pump1
    delay(200);
    if(pump2 != 0){dispense(2, pump2);}                     // Dispense with pump2
    delay(200);
    if(stirring != 0){motorStirring(stirring);}             // Stir the finished drink
    if(head){stepper.runToNewPosition(0);}                  // Raise stirring head
  
    consoleLog("Drink finished", 1);
  
    waitForRemove();
  }
  else
  {
    consoleLog("Not ready to serve drink", 2);
  }

}

// Handle pump manual commands received via MQTT
void handlePumps(String message, int pumpID) 
{
  if (message == "manual on") 
  {
    pump(pumpID, true);
  } 
  else if (message == "manual off") 
  {
    pump(pumpID, false);
  }
}

// Handle peltier manual commands received via MQTT
void handlePeltier(String message) 
{
  if (message == "manual on") 
  {
    peltier(true);
  } 
  else if (message == "manual off") 
  {
    peltier(false);
  }
}

// Handle motor manual commands received via MQTT
void handleMotor(String message) 
{
  if (message == "manual on") 
  {
    motor(true);
  } 
  else if (message == "manual off") 
  {
    motor(false);
  }
}

// Handle fan manual commands received via MQTT
void handleFan(String message) 
{
  if (message == "manual on") 
  {
    fan(true);
  } 
  else if (message == "manual off") 
  {
    fan(false);
  }
}