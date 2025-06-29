// ESP32 Automated Plant Watering Station code
//
// Written by Tamas Bozso for BTM Engineering
// Copyright (c) 2025 BTM Engineering
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Libraries ***********************************/
#include <WiFi.h>
#include <OneWire.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Espressif_Updater.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <HCSR04.h>
#include <SharpIR.h>
#include <Adafruit_MCP23X17.h>

#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "waterstation";
const char* currentSwVersion = "1.0.3";
const char* deviceModel = "ESP32-NodeMCU";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";
String  firmwareUrl;
String  latestSwVersion;
bool debugMode = false;

// Pin layout
/*  ESP32                      +-----------------------+
                               | O                   O |
                               |                       |
                          3.3V | [ ]               [ ] | GND
                            EN | [ ]  ___________  [ ] | GPIO23  
              IR SENSOR GPIO36 | [ ] |           | [ ] | GPIO22 SCL 
                        GPIO39 | [ ] |           | [ ] | GPIO01
                        GPIO34 | [ ] |           | [ ] | GPIO03
                        GPIO35 | [ ] |           | [ ] | GPIO21 SDA 
                        GPIO32 | [ ] |           | [ ] | GND
                        GPIO33 | [ ] |           | [ ] | GPIO19  
                        GPIO25 | [ ] |___________| [ ] | GPIO18  
                        GPIO26 | [ ]               [ ] | GPIO05  
                        GPIO27 | [ ]               [ ] | GPIO17 ECHO 
                   INTB GPIO14 | [ ]               [ ] | GPIO16 TRIGGER 
                        GPIO12 | [ ]               [ ] | GPIO04  
                           GND | [ ]               [ ] | GPIO00  
                   INTA GPIO13 | [ ]               [ ] | GPIO02  
                        GPIO09 | [ ]               [ ] | GPIO15  
                        GPIO10 | [ ]               [ ] | GPIO08
                        GPIO11 | [ ]               [ ] | GPIO07
                            5V | [ ]               [ ] | GPIO06
                               |        -------        |
                               | O      | USB |      O |
                               +-----------------------+         */

// Outputs 
const int UStriggerPin = 16; 
const int pumpOnePin = 27;
const int pumpTwoPin = 26;
const int floatPin = 19;                           

// Inputs
const int intAPin = 17;
const int intBPin = 16;
const int USechoPin = 17;
const int IRsensorPin = 36; 

/*  MCP23017                  +--------------+
                              | O          O |
                              |              |
                   SOIL1 GPB0 | [ ]      [ ] | GPA7 PUMP1
                   SOIL2 GPB1 | [ ]      [ ] | GPA6 PUMP2 
                   SOIL3 GPB2 | [ ]      [ ] | GPA5 LED 
                LIMIT UP GPB3 | [ ]      [ ] | GPA4 FAN
              LIMIT DOWN GPB4 | [ ]      [ ] | GPA3 
                         GPB5 | [ ]      [ ] | GPA2  
                         GPB6 | [ ]      [ ] | GPA1
                         GPB7 | [ ]      [ ] | GPA0  
                          Vdd | [ ]      [ ] | INTA  
                          Vss | [ ]      [ ] | INTB  
                           NC | [ ]      [ ] | RESET  
                          SCL | [ ]      [ ] | A2  
                          SDA | [ ]      [ ] | A1  
                           NC | [ ]      [ ] | A0  
                              |              |
                              | O          O |
                              +--------------+         */

// Outputs                              
const int ledPin = 5;
const int fanPin = 4;

// Inputs
const int soilOnePin = 8;
const int soilTwoPin = 9;
const int soilThreePin = 10;
const int limitUpPin = 13;
const int limitDownPin = 14;

// Status indicators
bool pumpStatus[] = {true, false, false, false, false, false, false};
bool statusMotor = false;
bool statusPeltier = false;
bool statusLED = false;
bool statusFan = true;

// Pump pin layout
int pumpPin[] = {99, pumpOnePin, pumpTwoPin, 99, 99, 99, 99};
  
// Refresh loop parameters
int refreshRate = 5000;                             // Measurement loop length
int refreshLoop = 1;                                // Number of refresh loops
double upTime = 0;
unsigned long previousMillis1 = 0;
unsigned long previousMillisMQTT = 0;               // MQTT reconnect timing
unsigned long previousMillisWiFi = 0;               // WiFi reconnect timing
unsigned long mqttReconnectInterval = 5000;         // Check MQTT every 5 seconds
unsigned long wifiReconnectInterval = 5000;         // Check WiFi every 5 seconds 
unsigned long wifiRetryMaxInterval = 30000;         // 30 seconds max
unsigned long mqttRetryMaxInterval = 60000;         // 60 seconds max 
bool wifiStatus = false;                            // WiFi status indicator
long wifiStrength;                                  // WiFi strength value 

// Calibration constants
double calibUsA = 0;
double calibUsB = 1;
double calibIrA = 0;
double calibIrB = 1;

SharpIR sensor( SharpIR::GP2Y0A41SK0F, IRsensorPin );
Adafruit_MCP23X17 mcp;

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
String fan_topic =                          String("home/") + deviceName + String("/fan");
String IRsensor_topic =                     String("home/") + deviceName + String("/IRsensor");
String USsensor_topic =                     String("home/") + deviceName + String("/USsensor");
String soil1_topic =                        String("home/") + deviceName + String("/moisture/soil1");
String soil2_topic =                        String("home/") + deviceName + String("/moisture/soil2");
String soil3_topic =                        String("home/") + deviceName + String("/moisture/soil3");
String float_topic =                        String("home/") + deviceName + String("/float");

// Numbers
String pump_time_state_topic =              String("home/") + deviceName + String("/pumps/time/state");
String pump_time_command_topic =            String("home/") + deviceName + String("/pumps/time/command");

// Switches
String pump1_state_topic =                  String("home/") + deviceName + String("/pumps/pump1/state");
String pump1_command_topic =                String("home/") + deviceName + String("/pumps/pump1/command");
String pump2_state_topic =                  String("home/") + deviceName + String("/pumps/pump2/state");
String pump2_command_topic =                String("home/") + deviceName + String("/pumps/pump2/command");

// Select
String sensor_mode_command_topic =          String("home/") + deviceName + String("/mode/command");
String sensor_mode_state_topic =            String("home/") + deviceName + String("/mode/state");

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
      subscribeTopic(pump1_command_topic);
      subscribeTopic(pump2_command_topic);
      subscribeTopic(pump_time_command_topic);

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
  else if (String(topic) == pump1_command_topic) 
  {
    handlePumps(message, 1);
  }
  else if (String(topic) == pump2_command_topic) 
  {
    handlePumps(message, 2);
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

  // Sensors
  publishMQTTSensorDiscovery("US Sensor", USsensor_topic, "mdi:storage-tank-outline", "cm", "distance", "measurement", "", 1);
  delay(100);
  publishMQTTSensorDiscovery("IR Sensor", IRsensor_topic, "mdi:storage-tank-outline", "cm", "distance", "measurement", "", 1);
  delay(100);

  // Binary Sensors
  publishMQTTBinarySensorDiscovery("LED", led_topic, "mdi:led-on", "light");
  delay(100);
  publishMQTTBinarySensorDiscovery("Fan", fan_topic, "mdi:fan","running");
  delay(100);
  publishMQTTBinarySensorDiscovery("Float Sensor", float_topic, "","problem");
  delay(100);

  // Numbers
  publishMQTTNumberDiscovery("Pump Time", pump_time_command_topic, pump_time_state_topic, 1.0, 60.0, 1.0, "mdi:timer", "s", true);
  delay(100);

  // Switches
  publishMQTTSwitchDiscovery("Pump1", pump1_command_topic, pump1_state_topic, "mdi:valve");
  delay(100);
  publishMQTTSwitchDiscovery("Pump2", pump2_command_topic, pump2_state_topic, "mdi:valve");
  delay(100);

  // Select
  publishMQTTSelectDiscovery("Measurement Strategy", sensor_mode_command_topic, sensor_mode_state_topic,{ "ultrasonic", "infrared", "average", "adaptive" },"mdi:tune");

  // Parameters
  publishMQTTSensorDiscovery("Refresh Rate", refreshRate_topic, "mdi:refresh", "", "", "", "diagnostic", 0);
  delay(100);
}

// Send back received parameters to the server
void sendParameters()
{
  publishMessage(refreshRate_topic, refreshRate, true);
}

/************************** Setup function ***********************************/

void setup(void)
{
  Serial.begin(115200);

  // Initialize pins
  pinMode(intAPin, INPUT);
  pinMode(intBPin, INPUT);
  pinMode(IRsensorPin, INPUT);
  pinMode(USechoPin, INPUT);
  pinMode(UStriggerPin, OUTPUT);
  pinMode(pumpOnePin, OUTPUT);
  pinMode(pumpTwoPin, OUTPUT);
  pinMode(floatPin, INPUT_PULLUP);

  digitalWrite(pumpOnePin, LOW);
  digitalWrite(pumpTwoPin, LOW);

  Wire.begin();
  scanI2CDevices(); // Check if MCP23017 responds

  // Initialize I2C and MCP23017
  mcp.begin_I2C(); // 0 = address 0x20 (A0-A2 = GND)

  // Initialize MCP23017 pins
  mcp.pinMode(ledPin, OUTPUT);
  mcp.pinMode(fanPin, OUTPUT);

  mcp.pinMode(soilOnePin, INPUT);
  mcp.pinMode(soilTwoPin, INPUT);
  mcp.pinMode(soilThreePin, INPUT);
  mcp.pinMode(limitUpPin, INPUT);
  mcp.pinMode(limitDownPin, INPUT);


  mcp.digitalWrite(ledPin, LOW);
  mcp.digitalWrite(fanPin, LOW);

  //mcp.pullUp(1, HIGH);

  // Initalize ultrasonic sensor
  HCSR04.begin(UStriggerPin, USechoPin);

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
}

/************************** Main loop ***********************************/

void loop(void)
{
  // Store the current computer time
  unsigned long currentMillis = millis();

  // Update connection status
  wifiStatus = (WiFi.status() == WL_CONNECTED);
  mqttStatus = client.connected();

  // Frequent WiFi reconnect attempt
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

  // Frequent MQTT reconnect attempt
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
  
  client.loop();

  // Standby loop
  if (currentMillis - previousMillis1 >= refreshRate * 1000) 
  { 
    previousMillis1 = currentMillis;

    // Uptime calculation
    refreshLoop++;
    upTime = (refreshLoop * refreshRate) / 60; // Return uptime in minutes
    Serial.print("Loop Number: ");
    Serial.println(refreshLoop);

    // Diagnostics
    wifiStrength = WiFi.RSSI();
    
    // Read sensors
    double USlevel = readLevelUltrasonic(5);
    double IRlevel = readLevelInfrared(5);
    bool floatSensor = readFloatSensor();

    Serial.println("Reading sensors:");
    Serial.print("  US Level: ");
    Serial.println(USlevel);
    Serial.print("  IR Level: ");
    Serial.println(IRlevel);
    Serial.print("  Float Sensor: ");
    Serial.println(floatSensor);

    // Send diagnostics to Home Assistant
    publishMessage(wifi_strength_topic, (double)wifiStrength, false);
    publishMessage(uptime_topic, upTime, false);

    // Send telemetry to Home Assistant
    publishMessage(USsensor_topic, USlevel, false);
    publishMessage(IRsensor_topic, IRlevel, false);
    publishMessage(float_topic, floatSensor ? "ON" : "OFF", false);
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
          publishMessage(pump1_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", true);
          break;
       case 2:
          publishMessage(pump2_state_topic, pumpStatus[pumpID] ? "ON" : "OFF", true);
          break;
       default:
         // statements
         break;
     }
}

void led(bool state)
{
    statusLED = state;
    
    if(state)
    {
      Serial.println("LED: START");
      digitalWrite(ledPin, HIGH); 
    }
    else 
    {
      Serial.println("LED: STOP");
      digitalWrite(ledPin, LOW);
    }

    publishMessage(led_topic, statusLED ? "ON" : "OFF", false); 
}

double readLevelUltrasonic(int numAvg)
{
  double sumDistance = 0.0;

  for (int i = 0; i < numAvg; ++i) 
  {
    double* distances = HCSR04.measureDistanceCm();
    if (distances != nullptr) {
      sumDistance += distances[0];
    }
    delay(100);
  }

  double average = sumDistance / numAvg;
  double distance = calibUsB * average + calibUsA;

  Serial.print("Ultrasonic distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

double readLevelInfrared(int numAvg)
{
  double sumDistance = 0.0;

  for (int i = 0; i < numAvg; ++i) 
  {
    double reading = sensor.getDistance();
    sumDistance += reading;
    delay(100);
  }

  double average = sumDistance / numAvg;
  double distance = calibIrB * average + calibIrA;

  Serial.print("Infrared distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

bool readFloatSensor ()
{
  int switchState = digitalRead(floatPin);

  return switchState;
}

double getWaterLevel(int numAvg)
{
  const double minValid = 5.0;   // cm
  const double maxValid = 50.0;  // cm
  const double maxDiff = 5.0;    // max allowed diff between sensors
  const double maxJump = 10.0;   // max allowed jump from previous reading

  static double prevLevel = -1.0;

  double ir = readLevelInfrared(numAvg);
  double us = readLevelUltrasonic(numAvg);

  bool irValid = (ir >= minValid && ir <= maxValid);
  bool usValid = (us >= minValid && us <= maxValid);

  double level = -1.0;

  if (irValid && usValid) {
    if (abs(ir - us) <= maxDiff) {
      level = (ir + us) / 2.0;
    } else {
      Serial.println("Sensor mismatch! Using ultrasonic as primary.");
      level = us;
    }
  } else if (usValid) {
    level = us;
  } else if (irValid) {
    level = ir;
  } else {
    Serial.println("Both sensor readings are invalid.");
    return prevLevel >= 0 ? prevLevel : -1.0;
  }

  // Compare with previous level
  if (prevLevel >= 0 && abs(level - prevLevel) > maxJump) {
    Serial.println("Sudden jump detected, using previous level instead.");
    return prevLevel;
  }

  prevLevel = level;
  return level;
}

void scanI2CDevices() 
{
  byte error, address;
  int nDevices = 0;

  Serial.println("🔍 Scanning I2C bus...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("✅ I2C device found at address 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("⚠️ Unknown error at 0x");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("❌ No I2C devices found. Check wiring!");
}


  }

  prevLevel = level;
  return level;
}


