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
#include <ESPmDNS.h>
#include <Update.h>
#include <Espressif_Updater.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "gasMeter";
const char* currentSwVersion = "1.0.2";
const char* deviceModel = "ESP32-NodeMCU";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";
String  firmwareUrl;
String  latestSwVersion;
bool debugMode = true;
  
// Refresh loop parameters
int refreshRate = 0.1;                                // Measurement loop length
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
String request_meter_topic =                String("home/") + deviceName + String("/meterState/request");
String receive_meter_topic =                String("home/") + deviceName + String("/meterState/receive");
String set_meter_topic =                    String("home/") + deviceName + String("/meterState/set");

// Sensors
String baseline_topic =                     String("home/") + deviceName + String("/baseline");
String mirrorline_topic =                   String("home/") + deviceName + String("/mirrorline");
String usage_topic =                        String("home/") + deviceName + String("/usage");
String pulse_topic =                        String("home/") + deviceName + String("/pulse");

// Numbers
String dropMargin_state_topic =            String("home/") + deviceName + String("/parameters/dropMargin/state");
String dropMargin_command_topic =          String("home/") + deviceName + String("/parameters/dropMargin/command");
String recoveryMargin_state_topic =        String("home/") + deviceName + String("/parameters/recoveryMargin/state");
String recoveryMargin_command_topic =      String("home/") + deviceName + String("/parameters/recoveryMargin/command");
String mirrorConfirm_state_topic =         String("home/") + deviceName + String("/parameters/mirrorConfirm/state");
String mirrorConfirm_command_topic =       String("home/") + deviceName + String("/parameters/mirrorConfirm/command");
String lockoutTime_state_topic =           String("home/") + deviceName + String("/parameters/lockoutTime/state");
String lockoutTime_command_topic =         String("home/") + deviceName + String("/parameters/lockoutTime/command");
String baselineAlpha_state_topic =         String("home/") + deviceName + String("/parameters/baselineAlpha/state");
String baselineAlpha_command_topic =       String("home/") + deviceName + String("/parameters/baselineAlpha/command");
String mirrorlineAlpha_state_topic =       String("home/") + deviceName + String("/parameters/mirrorlineAlpha/state");
String mirrorlineAlpha_command_topic =     String("home/") + deviceName + String("/parameters/mirrorlineAlpha/command");

// Buttons
String reset_topic =                       String("home/") + deviceName + String("/reset");

// Initalize the Mqtt client instance
WiFiClient espClient;
PubSubClient client(espClient);

// ======================================================
//                GAS METER – CONFIGURATION
// ======================================================

// -------------------- Runtime / Diagnostics --------------------
float upTime = 0;                                      // System uptime counter (seconds or minutes, user-defined)
static uint32_t lastPublishedPulse = 0;                // Last pulse count sent via MQTT
bool pulsePending = false;                             // Indicates pending incremental publish

// -------------------- Hardware --------------------
const int SENSOR_PIN = 34;                             // ADC1 pin only (ESP32 WiFi-safe ADC)

// -------------------- Timing --------------------
const unsigned long SENSOR_INTERVAL_MS = 50;          // ADC sampling interval
const unsigned long NET_INTERVAL_MS    = 30000;       // Network / diagnostics publish interval

unsigned long lastSensorMillis = 0;                   // Last ADC sampling timestamp
unsigned long lastNetMillis    = 0;                   // Last network update timestamp
unsigned long lastSample       = 0;                   // (Optional) legacy / debug timestamp

// -------------------- Gas Usage --------------------
float usageIncrement = 0.1;                           // Gas volume per pulse (m³ or unit-defined)
float meterState = 0;                                 // Absolute gas meter value (m³)

// -------------------- FSM State --------------------
enum State {
  NORMAL,                                             // No mirror detected – track baseline
  MIRROR,                                             // Mirror detected – wait for exit
  LOCKOUT                                             // Pulse registered – ignore noise
};

State state = NORMAL;                                 // Current FSM state

// -------------------- Optical Signal Levels --------------------
float baseline   = 0;                                 // Learned background reflectivity
float mirrorline = 0;                                 // Learned mirror reflectivity (diagnostic)

// -------------------- FSM Thresholds & Filters --------------------
int dropMargin     = 1200;                            // ADC drop below baseline to detect mirror entry
int recoveryMargin = 1000;                            // ADC rise above baseline to detect mirror exit
int mirrorConfirm  = 3;                               // Required consecutive low samples to confirm mirror
float baselineAlpha   = 0.01;                         // Low-pass filter for baseline tracking
float mirrorlineAlpha = 0.05;                         // Faster low-pass filter for mirror tracking
float lockoutTime = 800;                              // Minimum time (ms) between pulses (debounce)

// -------------------- Counters --------------------
int pulseCount = 0;                                   // Total detected pulses
int mirrorConfirmCount = 0;                           // Consecutive mirror detection counter

// -------------------- Timing (FSM) --------------------
unsigned long lockoutStart = 0;                       // Timestamp when LOCKOUT started

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
      subscribeTopic(dropMargin_command_topic);
      subscribeTopic(recoveryMargin_command_topic);
      subscribeTopic(mirrorConfirm_command_topic);
      subscribeTopic(lockoutTime_command_topic);
      subscribeTopic(baselineAlpha_command_topic);
      subscribeTopic(mirrorlineAlpha_command_topic);
      subscribeTopic(reset_topic);
      subscribeTopic(receive_meter_topic);

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
  else if (String(topic) == dropMargin_command_topic) 
  {
    handleParameter("dropMargin", message);
  }
  else if (String(topic) == recoveryMargin_command_topic) 
  {
    handleParameter("recoveryMargin", message);
  }
  else if (String(topic) == mirrorConfirm_command_topic) 
  {
    handleParameter("mirrorConfirm", message);
  }
  else if (String(topic) == lockoutTime_command_topic) 
  {
    handleParameter("lockoutTime", message);
  }
  else if (String(topic) == baselineAlpha_command_topic) 
  {
    handleParameter("baselineAlpha", message);
  }
  else if (String(topic) == mirrorlineAlpha_command_topic) 
  {
    handleParameter("mirrorlineAlpha", message);
  }
  else if (String(topic) == receive_meter_topic) 
  {
    handleParameter("meterState", message);
  }
  else if (String(topic) == reset_topic) 
  {
    consoleLog("Command received: Reboot", 1);
    restartESP();
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
  //String requestPayload = String("{\"device\":\"") + deviceName + String("\"}");
  //Serial.printf("[INFO] Publishing parameter request to MQTT: %s\n", requestPayload.c_str());
  //client.publish(parameter_request_topic.c_str(), requestPayload.c_str());
  publishMessage(request_meter_topic, true, false);
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

void publishMQTTAvailabilityBinarySensor(String name, String availabilityTopic)
{
  String uniqueID = String(deviceName) + "-" + removeSpaces(name);
  String objectID = String(deviceName) + "_" + removeSpaces(name);
  String topic = "homeassistant/binary_sensor/" + uniqueID + "/config";

  DynamicJsonDocument doc(512);
  doc["name"] = name;
  doc["unique_id"] = uniqueID;
  doc["object_id"] = objectID;
  doc["device_class"] = "connectivity";

  // This is key — we use availability topic for state!
  doc["state_topic"] = availabilityTopic;
  doc["payload_on"] = "connected";
  doc["payload_off"] = "connection lost";

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
  publishMQTTAvailabilityBinarySensor("Availability", availability_topic);
  delay(100);

  // Sensors
  publishMQTTSensorDiscovery("Baseline", baseline_topic, "mdi:chart-line-variant", "", "", "measurement", "", 1);
  delay(100);
  publishMQTTSensorDiscovery("Mirrorline", mirrorline_topic, "mdi:mirror", "", "", "measurement", "", 1);
  delay(100);
  publishMQTTSensorDiscovery("Current Meter", set_meter_topic, "mdi:gauge", "", "", "measurement", "", 3);
  delay(100);
  publishMQTTSensorDiscovery("Usage", usage_topic, "mdi:counter", "", "", "measurement", "", 3);
  delay(100);
  publishMQTTSensorDiscovery("Pulse Count", pulse_topic, "mdi:pulse", "", "", "measurement", "", -1);
  delay(100);

  // Numbers
  publishMQTTNumberDiscovery("Drop Margin", dropMargin_command_topic, dropMargin_state_topic, 0, 3000, 100, "mdi:chart-timeline-variant", "", true);
  delay(100);
  publishMQTTNumberDiscovery("Recovery Margin", recoveryMargin_command_topic, recoveryMargin_state_topic, 0, 3000, 100, "mdi:chart-timeline-variant-shimmer", "", true);
  delay(100);
  publishMQTTNumberDiscovery("Mirror Confirm", mirrorConfirm_command_topic, mirrorConfirm_state_topic, 0.0, 10.0, 1.0, "mdi:check-all", "", true);
  delay(100);
  publishMQTTNumberDiscovery("Lockout Time", lockoutTime_command_topic, lockoutTime_state_topic, 200, 1000, 50.0, "mdi:lock-clock", "", true);
  delay(100);
  publishMQTTNumberDiscovery("Baseline Alpha", baselineAlpha_command_topic, baselineAlpha_state_topic, 0.0, 0.1, 0.01, "mdi:alpha-a-box-outline", "", true);
  delay(100);
  publishMQTTNumberDiscovery("Mirrorline Alpha", mirrorlineAlpha_command_topic, mirrorlineAlpha_state_topic, 0.0, 0.1, 0.01, "mdi:alpha-a-box-outline", "", true);
  delay(100);

  // Buttons
  publishMQTTButtonDiscovery("Reboot ESP", reset_topic, "mdi:restart", false);
}

// Send back received parameters to the server
void sendParameters()
{
  publishMessage(dropMargin_state_topic, dropMargin, false);
  publishMessage(recoveryMargin_state_topic, recoveryMargin, false);
  publishMessage(mirrorConfirm_state_topic, mirrorConfirm, false);
  publishMessage(lockoutTime_state_topic, lockoutTime, false);
  publishMessage(baselineAlpha_state_topic, baselineAlpha, false);
  publishMessage(mirrorlineAlpha_state_topic, mirrorlineAlpha, false);
}

/************************** Setup function ***********************************/

void setup(void)
{
  Serial.begin(115200);

  // ADC configuration
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Read base line value
  baseline = analogRead(SENSOR_PIN);
  mirrorline = baseline - dropMargin;
  Serial.println("time_ms,adc,baseline,state,pulses");

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

  // Reset logging
  consoleLog("None", 1);
  consoleLog("None", 2);
  consoleLog("None", 3);

  // Publish diagnostic data to the server
  publishMessage(firmware_topic, currentSwVersion, true);
  publishMessage(log_info_topic, "Starting main loop", false);
  publishMessage(ip_topic, configurationUrl, true);
}

/************************** Main loop ***********************************/

void loop() 
{
  unsigned long now = millis();

  // ================= SENSOR + FSM (HIGH PRIORITY) =================
  if (now - lastSensorMillis >= SENSOR_INTERVAL_MS)
  {
    lastSensorMillis = now;

    // Read raw ADC value from IR photodiode
    int adc = analogRead(SENSOR_PIN);

    // ================= FSM =================
    switch (state)
    {
      // ------------------------------------------------------------
      // NORMAL STATE
      // ------------------------------------------------------------
      // Wheel is NOT showing the mirror.
      // Track the baseline reflectivity of the number wheel.
      case NORMAL:

        // Exponential moving average to learn baseline reflectivity
        baseline = baseline * (1.0 - baselineAlpha) + adc * baselineAlpha;

        // Detect potential mirror entry:
        // If signal drops significantly below baseline
        if (adc < baseline - dropMargin)
        {
          mirrorConfirmCount++;

          // Require multiple consecutive low samples
          // to avoid noise-triggered false transitions
          if (mirrorConfirmCount >= mirrorConfirm)
          {
            state = MIRROR;
            mirrorConfirmCount = 0;   // Reset counter on state change
          }
        }
        else
        {
          // Signal recovered → reset confirmation counter
          mirrorConfirmCount = 0;
        }
        break;

      // ------------------------------------------------------------
      // MIRROR STATE
      // ------------------------------------------------------------
      // Wheel mirror is in view.
      // Track mirror reflectivity and wait for exit.
      case MIRROR:

        // Learn mirror reflectivity separately
        mirrorline = mirrorline * (1.0 - mirrorlineAlpha) + adc * mirrorlineAlpha;

        // Mirror exit condition:
        // Signal rises back near baseline
        if (adc >= baseline - recoveryMargin)
        {
          pulseCount++;              // One full wheel step detected
          state = LOCKOUT;           // Prevent double-counting
          lockoutStart = now;
        }
        break;

      // ------------------------------------------------------------
      // LOCKOUT STATE
      // ------------------------------------------------------------
      // Short dead-time after a pulse to ignore bounce/noise
      case LOCKOUT:

        // Exit lockout after fixed time
        if (now - lockoutStart >= lockoutTime)
        {
          state = NORMAL;
        }
        break;
    }

    // ================= INCREMENTAL PUBLISH LOGIC =================
    // Trigger data transmission every 10 pulses (e.g. 0.01 m³)
    if (pulseCount - lastPublishedPulse >= 10)
    {
      pulsePending = true;
      lastPublishedPulse = pulseCount;
    }

    // ================= DEBUG OUTPUT =================
    // USB-only debug (safe: no Wi-Fi, no timing impact)
    if (debugMode)
    {
      Serial.print("ADC ");
      Serial.print(adc);
      Serial.print(" BL ");
      Serial.print((int)baseline);
      Serial.print(" ML ");
      Serial.print((int)mirrorline);
      Serial.print(" ST ");
      Serial.print(state);
      Serial.print(" PC ");
      Serial.println(pulseCount);
    }
  }

  // ================= INCREMENTAL UPDATE (MED PRIORITY) =================
  if (pulsePending && client.connected()) 
  {
    pulsePending = false;

    // Update absolute meter state
    meterState = meterState + usageIncrement;
    
    // Send increment data
    publishMessage(usage_topic, usageIncrement, false);
    publishMessage(set_meter_topic, meterState, false);
  }

  // ================= NETWORK / DIAGNOSTICS (LOW PRIORITY) =================
  if (now - lastNetMillis >= NET_INTERVAL_MS) 
  {
    lastNetMillis = now;

    // WiFi maintenance
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WARNING] WiFi Disconnected! Attempting to reconnect...");
      if (connectWifi())  
      { 
        Serial.println("[WARNING] WiFi Reconnection Successfull");
      }
      else
      { 
        Serial.println("[WARNING] WiFi Reconnecton Failed!");
      }
    }

    // MQTT maintenance
    if (!client.connected()) {
      Serial.println("[WARNING] MQTT Disconnected! Attempting to reconnect...");
      if (connectMQTT())  
      { 
        Serial.println("[WARNING] MQTT Reconnection Successfull");
      }
      else
      { 
        Serial.println("[WARNING] MQTT Reconnecton Failed!");
      }
    }
    // Announce availability
    publishMessage(availability_topic, "connected", true);

    // Diagnostics
    upTime += (float)NET_INTERVAL_MS / 3600000.0;
    wifiStrength = WiFi.RSSI();

    publishMessage(wifi_strength_topic, (double)wifiStrength, false);
    publishMessage(baseline_topic, baseline, false);
    publishMessage(mirrorline_topic, mirrorline, false);
    publishMessage(pulse_topic, pulseCount, false);
    publishMessage(uptime_topic, upTime, false);
  }

  client.loop();
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
void handleParameter(String parameterName, String value) 
{
  if (parameterName == "dropMargin") 
  {
    dropMargin = value.toInt();
  }
  else if (parameterName == "recoveryMargin") 
  {
    recoveryMargin = value.toInt();
  }
  else if (parameterName == "mirrorConfirm") 
  {
    mirrorConfirm = value.toInt();
  }
  else if (parameterName == "lockoutTime") 
  {
    lockoutTime = value.toInt();
  }
  else if (parameterName == "baselineAlpha") 
  {
    baselineAlpha = value.toFloat();
  }
  else if (parameterName == "mirrorlineAlpha") 
  {
    mirrorlineAlpha = value.toFloat();
  }
  else if (parameterName == "meterState") 
  {
    meterState = value.toFloat();
  }
  else 
  {
    consoleLog("Unknown parameter: " + parameterName, 2);
    return;
  }

  consoleLog("Parameter '" + parameterName + "' set to " + value, 1);
}
