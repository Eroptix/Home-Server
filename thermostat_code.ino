// ESP32 Thermostat control code
//
// Written by Tamas Bozso for BTM Engineering
// Copyright (c) 2021 BTM Engineering
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Libraries ***********************************/

#include <EEPROM.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#if defined(ARDUINO_ARCH_ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Time.h>
#include "time.h"
#include <array>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "thermostat";
const char* currentSwVersion = "1.3.5";
const char* deviceModel = "ESP32-NodeMCU";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";
String  firmwareUrl;
String  latestSwVersion;

// Wi-Fi credentials
const char* WIFI_SSID = "UPC8F21BEF";
const char* WIFI_PASS = "k7pp3aexkmQh";

// EEPROM Addresses
int SERVO_ADDRESS = 0;                              // EEPROM address to store last servo angle
int MODE_ADDRESS = 1;                               // EEPROM address to store last active mode 
int TEMP_ADDRESS = 2;                               // EEPROM address to store last goal temperature setting

// Measurement variables
float celsius;
float hum;
float pres;

// Calibration constants
double tempCalibration = -3.2;                      // Calibration constant for temperature readings
double presCalibration = 0;                         // Calibration constant for pressure readings                     
double humCalibration = 26;                         // Calibration constant for humidity readings

// Constants to convert selected temperature to servo position
double servoConvertA = 5.817;                       // Temperature - angle function linear fit A value
double servoConvertB = -25.466;                     // Temperature - angle function linear fit B value
int previousAngle;                                  // Previous angle setting
int servoLastAngle = 90;                            // Last servo angle before reboot

// Refresh loop parameters
int refreshRate = 60;                       // Measurement loop length [s]
int refreshLoop = 1;                                // Number of refresh loops
int connectRate = 300;                              // Connection check loop length [s]
bool manualTrigger = false;  
unsigned long previousMillisMain = 0;
unsigned long previousMillisMQTT = 0;               // MQTT reconnect timing
unsigned long previousMillisWiFi = 0;               // WiFi reconnect timing
unsigned long mqttReconnectInterval = 5000;   // Check MQTT every 5 seconds
unsigned long wifiReconnectInterval = 5000;   // Check WiFi every 5 seconds 
unsigned long wifiRetryMaxInterval = 30000;    // 30 seconds max
unsigned long mqttRetryMaxInterval = 60000;    // 60 seconds max 


// Temperature control parameters
int onTemperature = 22;                             // Temperature setting for heating on state
int offTemperature = 14;                            // Temperature setting for heating off state
double tempControlRange = 0.15;                     // Temperature control range for on/off switching
int heaterSetting = offTemperature;                 // Current temperature setting for the heating logic (onTemperature or offTemperature)
int tempGoal;                                       // Goal temperature setting for the heating logic
int safetyTemp = 18;                                // Safety temperature when device loses connection
bool autoMode = false;                              // Auto mode on/off state bool
bool lcdMode = true;                                // LCD on/off state bool
bool wifiStatus = false;                            // WiFi status indicator
long wifiStrength;                                  // WiFi strength value
String mode = "heat";                               // Active temperature control mode (auto, heat, cool, off) 

// Default temperature schedule
//[01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
int tempSchedule[7][24] = 
{
  {18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 19, 18},  // Monday
  {18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 19, 18},  // Tuesday
  {18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 19, 18},  // Wednesday
  {18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 19, 18},  // Thursday
  {18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20},  // Friday
  {20, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20},  // Saturday
  {20, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 19, 18}   // Sunday
};

// JSON format for Home Assistant automation payload
/*
    [DAY]       [00][01][02][03][04][05][06][07][08][09][10][11][12][13][14][15][16][17][18][19][20][21][22][23]
{
  "Monday":     [18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 19, 18],
  "Tuesday":    [18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 19, 18],
  "Wednesday":  [18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 19, 18],
  "Thursday":   [18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 19, 18],
  "Friday":     [18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20],
  "Saturday":   [20, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20],
  "Sunday":     [20, 18, 18, 18, 18, 18, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 19]
}
*/

//Servo movement parameters
int angleIncrement = 1;
int incrementDelay = 75;

Adafruit_BME280 bme;
Servo servo;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

#define SERVO_PIN 2
#define EEPROM_SIZE 30

/************************** Clock Setup ***********************************/

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org");
ESP32Time rtc(3600);  // offset in seconds GMT+1

// Variables to save date and time
String formattedDate;
String dayStamp;
int timeDay;
int timeStamp;
int timeMinutes;
int timeOffset = 1;
double timeHour;
int upTime;
const char* ntpServer = "europe.pool.ntp.org";
long  gmtOffset_sec = 0;
int   daylightOffset_sec = 3600;

/************************** LCD Icons ***********************************/

byte termometru[8]={B00100, B01010, B01010, B01110, B01110, B11111, B11111, B01110};    // Icon for termometer
byte picatura[8]={B00100, B00100, B01010, B01010, B10001, B10001, B10001, B01110};      // Icon for water droplet
byte wifiON[8] = {B00000, B00000, B00001, B00011, B00111, B01111, B11111, B00000};      // Icon for wifi ON
byte wifiOFF[8] = {B10100, B01000, B10100, B00001, B00011, B00111, B01111, B11111};     // Icon for wifi OFF
byte heatOn[8] = {B00000, B10101, B10101, B10101, B10101, B00000, B11111, B10101};      // Icon for heating ON
byte heatOff[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B11111, B10101};     // Icon for heating OFF

/************************** HomeAssistant Settings ***********************************/

// MQTT broker details
const char* mqtt_server = "192.168.0.241";    // IP address of the MQTT broker
const int mqtt_port = 1783;                   // Port number of the MQTT broker
const char* mqtt_client_id = deviceName;      // Client ID for this device 
bool mqttStatus = false;                      // MQTT connection status

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
String ip_topic =                           String("home/") + deviceName + String("/wifi/ip");
String wifi_strength_topic =                String("home/") + deviceName + String("/wifi/strength");

// Sensors
String temperature_topic =                  String("home/") + deviceName + String("/temperature");
String humidity_topic =                     String("home/") + deviceName + String("/humidity");
String pressure_topic =                     String("home/") + deviceName + String("/pressure");
String tempgoal_topic =                     String("home/") + deviceName + String("/tempgoal");
String mode_topic =                         String("home/") + deviceName + String("/mode");
String heating_topic =                      String("home/") + deviceName + String("/heating");

// Climate
String climate_temperature_command_topic =  String("home/") + deviceName + String("/climate/target_temperature/set");
String climate_mode_command_topic =         String("home/") + deviceName + String("/climate/mode/set");
String climate_temperature_state_topic =    String("home/") + deviceName + String("/climate/target_temperature");
String climate_mode_state_topic =           String("home/") + deviceName + String("/climate/mode");
String climate_temperature_current_topic =  String("home/") + deviceName + String("/climate/current_temperature");
String climate_schedule_request_topic =     String("home/") + deviceName + String("/climate/schedule/request");
String climate_schedule_response_topic =    String("home/") + deviceName + String("/climate/schedule");

// Parameters
String ontemperature_topic =                String("home/") + deviceName + String("/parameters/ontemperature");
String offtemperature_topic =               String("home/") + deviceName + String("/parameters/offtemperature");
String tempcontrolrange_topic =             String("home/") + deviceName + String("/parameters/tempcontrolrange");
String safetytemp_topic =                   String("home/") + deviceName + String("/parameters/safetytemp");
String refreshRate_topic =          String("home/") + deviceName + String("/parameters/refreshRate");
String timeoffset_topic =                   String("home/") + deviceName + String("/parameters/timeoffset");
String tempoffset_topic =                   String("home/") + deviceName + String("/parameters/tempoffset");

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
      subscribeTopic(climate_mode_command_topic);
      subscribeTopic(climate_temperature_command_topic);
      subscribeTopic(climate_schedule_response_topic);

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
  if (String(topic) == parameter_response_topic) 
  {
    handleParameters(message);
  }
  if (String(topic) == command_topic) 
  {
    handleCommand(message);
  }
  if (String(topic) == climate_temperature_command_topic) 
  {
    handleTemperature(message);
  }
  if (String(topic) == climate_mode_command_topic) 
  {
    handleMode(message);
  }
  if (String(topic) == climate_schedule_response_topic) 
  {
    handleTemperatureProgram(message);
  }

  // Trigger next measurement loop
  triggerLoop();
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
    Serial.print("JSON deserialization failed: ");
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
  Serial.println("[INFO] Starting OTA update...");
  publishMessage(ota_status_topic.c_str(), "Updating", true);
  
  // Resource optimization: Check heap memory
  if (ESP.getFreeHeap() < 20000) {
    Serial.println("[ERROR] Not enough memory for OTA. Aborting.");
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
        Serial.println("[INFO] OTA update successful!");
        Serial.print("		From: ");
        Serial.println(currentSwVersion);
        Serial.print("		To: ");
        Serial.println(latestSwVersion);
        Serial.print("		Size: ");
        Serial.println(written);
        Serial.println("[INFO] Restarting device in 10 seconds");
        publishMessage(ota_status_topic.c_str(), "Updated", true);
        delay(10000);
        ESP.restart();
      } else {
        Serial.println("[ERROR] OTA update failed.");
        Update.printError(Serial);
        publishMessage(ota_status_topic.c_str(), "Failed", true);
      }
    } else {
      Serial.println("[ERROR] Not enough space for OTA update.");
      publishMessage(ota_status_topic.c_str(), "Failed", true);
    }
  } else {
    Serial.printf("[ERROR] HTTP request failed with code: %d\n", httpCode);
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
    Serial.println("[INFO] Firmware update available! Starting OTA update");
    lcdUpdate(currentSwVersion,latestSwVersion);
    performOTA();
  } else {
    Serial.println("[INFO] Device firmware is up to date.");
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

// Request server-side temperature schedule from Home Assistant
void requestTemperatureSchedule()
{
  String requestPayload = String("{\"device\":\"") + deviceName + String("\"}");
  Serial.printf("[INFO] Publishing temperature schedule request to MQTT: %s\n", requestPayload.c_str());
  client.publish(climate_schedule_request_topic.c_str(), requestPayload.c_str());
}

// Remove white spaces from a String input
String removeSpaces(String input) {
  String output = "";
  for (int i = 0; i < input.length(); i++) {
    if (input[i] != ' ') {  
      output += input[i];  // Append only non-space characters
    }
  }
  return output;
}

// Create default discovery payload
void publishMQTTDiscovery(String name, String deviceType,	String icon, String unitOfMeasurement, String deviceClass, String stateClass, String entityCategory, String stateTopic) 
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

// Create a specific climate discovery payload
void publishMQTTClimateDiscovery(String name, String deviceType) 
{
    // Construct IDs
    String uniqueID = "climate-" + String(deviceName);
    String objectID = "climate_" + String(deviceName);
    
    // Construct discovery topic
    String topic = "homeassistant/" + deviceType + "/" + uniqueID + "/config";
    
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

// Send discovery topics to Home Assistant
void sendDiscoveries()
{
	// Diagnostics
  publishMQTTDiscovery("Up Time", "sensor","mdi:clock", "h", "duration", "total_increasing", "diagnostic", uptime_topic);
	publishMQTTDiscovery("OTA Status", "sensor", "mdi:update", "", "", "", "diagnostic", ota_status_topic);
	publishMQTTDiscovery("Firmware Version", "sensor", "mdi:application-outline", "", "", "", "diagnostic", firmware_topic);
	publishMQTTDiscovery("Error", "sensor", "mdi:alert-circle-outline", "", "", "", "diagnostic", log_error_topic);
	publishMQTTDiscovery("Warning", "sensor", "mdi:shield-alert-outline", "", "", "", "diagnostic", log_warning_topic);
	publishMQTTDiscovery("Info", "sensor", "mdi:information-outline", "", "", "", "diagnostic", log_info_topic);
	publishMQTTDiscovery("IP Address", "sensor", "mdi:ip-network-outline", "", "", "", "diagnostic", ip_topic);
  publishMQTTDiscovery("WiFi Strength", "sensor", "mdi-rss", "", "", "", "diagnostic", wifi_strength_topic);
  // Sensors
  publishMQTTDiscovery("Temperature", "sensor", "mdi:home-thermometer", "°C", "temperature", "measurement", "", temperature_topic);
  publishMQTTDiscovery("Pressure", "sensor", "mdi:gauge", "hPa", "pressure", "measurement", "", pressure_topic);
  publishMQTTDiscovery("Humidity", "sensor", "mdi:water-percent", "%", "humidity", "measurement", "", humidity_topic);
  publishMQTTDiscovery("Temp Goal", "sensor", "mdi:target", "°C", "temperature", "measurement", "", tempgoal_topic);
  publishMQTTDiscovery("Mode", "sensor", "mdi:auto-mode", "", "", "", "", mode_topic);
  publishMQTTDiscovery("Heating", "sensor", "mdi:heat-wave", "", "", "", "", heating_topic);
  // Parameters
  publishMQTTDiscovery("On Temperature", "sensor", "mdi:fire", "°C", "temperature", "measurement", "diagnostic", ontemperature_topic);
  publishMQTTDiscovery("Off Temperature", "sensor", "mdi:snowflake-alert", "°C", "temperature", "measurement", "diagnostic", offtemperature_topic);
  publishMQTTDiscovery("Control Range", "sensor", "mdi:car-cruise-control", "", "", "", "diagnostic", tempcontrolrange_topic);
  publishMQTTDiscovery("Safety Temperature", "sensor", "mdi:seatbelt", "°C", "temperature", "measurement", "diagnostic", safetytemp_topic);
  publishMQTTDiscovery("Refresh Rate", "sensor", "mdi:refresh", "", "", "", "diagnostic", refreshRate_topic);
  publishMQTTDiscovery("Time Offset", "sensor", "mdi:clock-time-eight-outline", "", "", "", "diagnostic", timeoffset_topic);
  publishMQTTDiscovery("Temperature Offset", "sensor", "mdi:thermometer-chevron-up", "", "", "", "diagnostic", tempoffset_topic);
  // Climate
  publishMQTTClimateDiscovery("thermostat", "climate"); 
}

// Send back received parameters to the server
void sendParameters()
{
  publishMessage(ontemperature_topic, onTemperature, true);
  publishMessage(offtemperature_topic, offTemperature, true);
  publishMessage(tempcontrolrange_topic, tempControlRange, true);
  publishMessage(safetytemp_topic, safetyTemp, true);
  publishMessage(refreshRate_topic, refreshRate, true);
  publishMessage(timeoffset_topic, timeOffset, true);
  publishMessage(tempoffset_topic, tempCalibration, true);
}

/************************** Setup function ***********************************/

void setup() 
{

  // start the serial connection
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Firmware version: ");
  Serial.println(currentSwVersion); 

  // Connect to WiFi network
  connectWifi();

  // Initialize BME280
  bme.begin(0x76);

  //Initialize Servo
  servoLastAngle = EEPROM.read(SERVO_ADDRESS);
  if (servoLastAngle == 255) servoLastAngle = 32;
  previousAngle=servoLastAngle;
  
  Serial.print("Last servo angle read: ");
  Serial.println(servoLastAngle);
  
  servo.write(servoLastAngle);
  servo.attach(SERVO_PIN);

  // Initiate the LCD:
  Serial.println("Initializing LCD display");
  lcd.init();
  lcd.createChar(1, termometru);
  lcd.createChar(2, picatura);
  lcd.createChar(3, wifiON);
  lcd.createChar(4, wifiOFF);
  lcd.createChar(5, heatOn);
  lcd.createChar(6, heatOff);
  lcd.backlight();

  // Display current firmware version
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FIRMWARE VERSION");
  lcd.setCursor(5,1);
  lcd.print(currentSwVersion);

  delay(3000);

  // Set MQTT server and callback
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(4096);
  client.setKeepAlive(120);
  connectMQTT();

  // Announce availability
  publishMessage(availability_topic, "connected", true);

  // Query latest firmware version
  requestFirmwareVersion();
  delay(1000);

  // Request boot parameters
  requestParameters();
  delay(1000);

  // Request temperature schedule
  requestTemperatureSchedule();
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

  // Publish data to the server
  publishMessage(firmware_topic, currentSwVersion, true);
  publishMessage(log_info_topic, "Starting main loop", false);
  publishMessage(ip_topic, configurationUrl, true);

  // Print connection status to LCD
  Serial.println("Printing boot status to LCD display");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(WiFi.localIP());
  lcd.setCursor(0,5);  
  lcd.print("BOOT: ");
  lcd.print(wifiStatus);
  lcd.print(" ");
  lcd.print(mqttStatus);
  lcd.print(" ");

  // Configure clock
  Serial.println("Initializing internal clock");
  gmtOffset_sec = timeOffset * 3600;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //printLocalTime();

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)){
    rtc.setTimeStruct(timeinfo); 
  }

  // Get time with RTC
  timeStamp = rtc.getHour(true);
  timeMinutes = rtc.getMinute();
  Serial.print("Time: ");
  Serial.print(timeStamp);
  Serial.print(" : ");
  Serial.println(timeMinutes);

  // Load temperature
  if(EEPROM.read(TEMP_ADDRESS) != 0)
  {
    // Load previous temperature goal
    tempGoal = EEPROM.read(TEMP_ADDRESS);
  }
  else 
  {
    // First boot after flashing
    tempGoal = 20;    
  }

  // Load mode
  autoMode = EEPROM.read(MODE_ADDRESS);
               
  delay(3000);

  // Trigger next measurement loop
  triggerLoop();

}

/************************** Main loop ***********************************/

void loop() 
{

  // Store the current computer time
  unsigned long currentMillis = millis();

  client.loop();

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

  if (manualTrigger || (currentMillis - previousMillisMain >= refreshRate * 1000)) 
  { 
    previousMillisMain = currentMillis;
    manualTrigger = false;

    // Uptime calculation
    refreshLoop++;
    upTime = (refreshLoop * refreshRate) / 60; // Return uptime in minutes
    Serial.print("Loop Number: ");
    Serial.println(refreshLoop);

    // Get time information
    timeDay = rtc.getDayofWeek() - 1;
    timeStamp = rtc.getHour(true);
    timeMinutes = rtc.getMinute();
    Serial.printf("Day: %d  Time: %02d:%02d\n", timeDay, timeStamp, timeMinutes);

    // Read temperature data from BME280
    celsius = bme.readTemperature() + tempCalibration;     
    Serial.print("Celsius: ");
    Serial.print(celsius);
    Serial.println(" C");
  
    // Read humidity data from BME280
    hum = bme.readHumidity() + humCalibration;
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.println(" %");
    
    // Read pressure data from BME280
    pres = (bme.readPressure()/ 100.0F) + presCalibration;       
    Serial.print("Pressure: ");
    Serial.print(pres);
    Serial.println(" hPa");

    if (!client.connected()) 
    {
      Serial.println("--------------------------------------");  
      Serial.println("MQTT connection: Not Connected");

      // Set servo if automatic mode on
      if (mode == "auto")
      {
        tempGoal = tempSchedule[timeDay][timeStamp];
      }

      // In safe mode -> set temperature to latest tempGoal
      setTemperature(tempGoal,celsius);
      
      // LCD Screen refresh
      lcdRefresh("OFFLINE",timeStamp,timeMinutes, celsius, hum, heaterSetting, tempGoal, mode);
    }
    else
    {
      Serial.println("--------------------------------------");
      Serial.println("MQTT connection: Connected");

      wifiStrength = WiFi.RSSI();

      // Send diagnostic data to the server
      publishMessage(wifi_strength_topic, (double)wifiStrength, false);
      publishMessage(uptime_topic, upTime, false);
      publishMessage(availability_topic, "connected", false);

      // Send telemetry data to the server
      publishMessage(temperature_topic, celsius, false);
      publishMessage(humidity_topic, hum, false);
      publishMessage(pressure_topic, pres, false);
      publishMessage(tempgoal_topic, tempGoal, false);
      publishMessage(mode_topic, mode, false);   
    
      // Check for active mode
      if (mode == "auto")
      {
        tempGoal = tempSchedule[timeDay][timeStamp];
      }

      Serial.print("Temperature Goal: ");
      Serial.println(tempGoal);

      // Set heating 
      setTemperature(tempGoal,celsius);

      // Send climate entity data
      publishMessage(climate_temperature_current_topic, celsius, false);
      publishMessage(climate_temperature_state_topic, tempGoal, false);
      publishMessage(climate_mode_state_topic, mode, false);

      // Refresh LCD Screen
      lcdRefresh("ONLINE",timeStamp,timeMinutes, celsius, hum, heaterSetting, tempGoal, mode);

      Serial.println("--------------------------------------");
    }
  }

  // Add delay to the loop
  delay(200);
}

/************************** Device Functions ***********************************/

// Open WiFi connection
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
      wifiStatus = false;
    }
  }
  
  configurationUrl = WiFi.localIP().toString();

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  wifiStatus = true;

  return wifiStatus;
}

// Restart ESP device
void restartESP()
{
  Serial.println("Rebooting ESP32");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("REBOOTING");
  delay(2000);
  esp_restart();
}

// Servo movement logic
void servoMove (int turnAngle) 
{
  // Check if servo is detached
  if(servo.attached() == false)
  {
    servo.write(previousAngle);
    servo.attach(SERVO_PIN);
  }

  //This function moves the servo slower with an adjustable angle increment and delay  
  if(turnAngle>previousAngle)
  {
    for(int i=previousAngle; i<=turnAngle; i+=angleIncrement)
    {
      servo.write(i);
      delay(incrementDelay);
    }
  }

  if(turnAngle<previousAngle)
  {
    for(int i=previousAngle; i>=turnAngle; i-=angleIncrement)
    {
      servo.write(i);
      delay(incrementDelay);
    }
  }

  // Save current angle 
  previousAngle = turnAngle;

  // Detach servo after movement to prevent noise
  //delay(500);  // Allow it to stabilize before detaching
  //servo.detach();
}

// Temperature control logic
void temperatureControl(float currentTemp, int setTemp) 
{    
    //This function contains the temperature control logic
    if(currentTemp > setTemp + tempControlRange) 
    {
      //Current temperature higher than set temperature range --> OFF
      heaterSetting = offTemperature;
    }

    if(currentTemp < setTemp - tempControlRange) 
    {
      //Current temperature lower than set temperature range --> ON
      heaterSetting = onTemperature;
    }
}

// Set goal temperature and move servo
void setTemperature(int tempGoal, float celsius)
{
  //Heater control logic to modify heatingSetting parameter
  temperatureControl(celsius, tempGoal);
  
  Serial.print("Heater Setting: ");
  Serial.println(heaterSetting);
  
  //Send heater setting
  if(heaterSetting == onTemperature)
  {
    publishMessage(heating_topic, true, false);
  }
  else 
  {
    publishMessage(heating_topic, false, false);
  }
  
  //Convert the selected temperature to servo position
  int angle = servoConvertA * heaterSetting + servoConvertB;
  
  Serial.print("Servo Angle: ");
  Serial.println(angle);
  
  //Check if servo position is valid
  if(angle < 5) 
  {
    angle = 5;
  }
  else if(angle > 160) 
  {
    angle = 160;
  }
  
  //Send order to the servo
  servoMove(angle);
  
  //Save last servo position to the EEPROM
  EEPROM.write(SERVO_ADDRESS, angle);
  EEPROM.commit();
}

// Refresh LCD display
void lcdRefresh(String connectionStatus, int timeHour, int timeminutes, double temperature, double humidity, int heatingSetting, int temperatureGoal, String mode)
{
        // LCD Screen refresh
        lcd.clear();
        lcd.setCursor(0,0);
        if(timeHour < 10) {lcd.print(" ");}
        lcd.print(timeHour);
        lcd.print(":");
        if(timeminutes < 10) {lcd.print("0");}
        lcd.print(timeminutes); 
        lcd.print(" ");
        if(connectionStatus == "OFFLINE"){lcd.print(char(4));}
        else {lcd.print(char(3));}
        lcd.print("  ");
        lcd.print(char(2));
        lcd.print(" ");
        lcd.print(humidity,1);
        lcd.print("%"); 
        lcd.setCursor(0,1);
        if(mode == "auto")
        {
          lcd.print("A");
        }
        else if(mode == "heat") 
        {
          lcd.print("M");
        }
        else
        {
          lcd.print("C");
        }
        lcd.print("|");
        lcd.print(temperatureGoal);
        lcd.print("| ");
        if(heatingSetting == onTemperature){lcd.print(char(5));}
        else {lcd.print(char(6));}
        lcd.print("  ");
        lcd.print(char(1));
        lcd.print(" ");
        lcd.print(temperature,1);
        lcd.print((char)223);
}

// Display update LCD screen
void lcdUpdate(String currentFW, String latestFW)
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("UPDATING FW:");
  lcd.setCursor(0,1);
  lcd.print(" ");
  lcd.print(currentFW);
  lcd.print(" -> " );
  lcd.print(latestFW);
}

// Display MQTT command LCD screen
void lcdCommand(String message)
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MQTT MESSAGE:");
  lcd.setCursor(0,1);
  lcd.print(" ");
  lcd.print(message);
  delay(5000);
}

// Print local time for debugging
void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");
}

// Triggers main measurement loop
void triggerLoop()
{
  // Trigger next measurement loop
  manualTrigger = true;
}

// Handle target temperature received via MQTT
void handleTemperature(String setTemp)
{  
  int newTempGoal = setTemp.toInt();

  if (newTempGoal == tempGoal) 
  {
    return;
  }

  tempGoal = newTempGoal;
  
  // Turn off auto mode
  handleMode("heat");
  
  // Save temperature goal to EEPROM
  EEPROM.write(TEMP_ADDRESS, tempGoal);
  EEPROM.commit();

  Serial.print("    [RPC] Set new temperature goal: ");
  Serial.println(tempGoal);

  publishMessage(tempgoal_topic, tempGoal, false);

  setTemperature(tempGoal,celsius);
}

// Handle modes received via MQTT
void handleMode(String setMode)
{
  if(setMode == "auto")
  {
    autoMode = true;
    mode = "auto";
    Serial.print("    [RPC] Mode: ");
    Serial.println(autoMode);

    // Save mode setup to EEPROM
    EEPROM.write(MODE_ADDRESS, autoMode);
    EEPROM.commit();

    // Just an response example
    publishMessage(mode_topic, mode, false);
    publishMessage(climate_mode_state_topic, mode, false);
  }
  if(setMode == "heat")
  {
    autoMode = false;
    mode = "heat";
    Serial.print("    [RPC] Mode: ");
    Serial.println(autoMode);

    // Save mode setup to EEPROM
    EEPROM.write(MODE_ADDRESS, autoMode);
    EEPROM.commit();

    // Just an response example
    publishMessage(mode_topic, mode, false);
    publishMessage(climate_mode_state_topic, mode, false);
  }
  if(setMode == "cool")
  {
    autoMode = false;
    mode = "cool";
    Serial.print("    [RPC] Mode: ");
    Serial.println(autoMode);

    // Save mode setup to EEPROM
    EEPROM.write(MODE_ADDRESS, autoMode);
    EEPROM.commit();

    // Just an response example
    publishMessage(mode_topic, mode, false);
    publishMessage(climate_mode_state_topic, mode, false);
  }
}

// Handle commands received via MQTT
void handleCommand(String message) 
{
  if (message == "reboot") 
  {
    Serial.println("Command received: Reboot");
    lcdCommand("reboot");
    restartESP();
  }
  if (message == "request parameters") 
  {
    Serial.println("Command received: Parameter Request");
    lcdCommand("rq parameter");
    requestParameters();
  }
  if (message == "send parameters") 
  {
    Serial.println("Command received: Parameter Send");
    lcdCommand("sd parameter");
    sendParameters();
  }
  if (message == "send discoveries") 
  {
    Serial.println("Command received: Discovery Send");
    lcdCommand("sd discovery");
    sendDiscoveries();
  }
  if (message == "servo off") 
  {
    Serial.println("Command received: Servo Off");
    lcdCommand("servo off");
    servo.detach();
  }
  if (message == "request schedule") 
  {
    Serial.println("Command received: Schedule Request");
    lcdCommand("rq schedule");
    requestTemperatureSchedule();
  }
  if (message == "lcd on") 
  {
    Serial.println("Command received: LCD Display ON");
    lcdCommand("lcd on");
    requestTemperatureSchedule();
  }
  if (message == "lcd off") 
  {
    Serial.println("Command received: LCD Display OFF");
    lcdCommand("lcd off");
    requestTemperatureSchedule();
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
        Serial.print("JSON Parsing Failed: ");
        Serial.println(error.c_str());
        return;
    }

    // Check if all expected parameters exist before assigning values
    if (!doc.containsKey("onTemperature") ||
        !doc.containsKey("offTemperature") ||
        !doc.containsKey("tempControlRange") ||
        !doc.containsKey("safetyTemp") ||
        !doc.containsKey("refreshRate") ||
        !doc.containsKey("timeOffset") ||
        !doc.containsKey("tempOffset")) 
    {
        Serial.println("Error: Missing one or more required parameters.");
        return;
    }

    // Extract values
    onTemperature = doc["onTemperature"].as<int>();
    offTemperature = doc["offTemperature"].as<int>();
    tempControlRange = doc["tempControlRange"].as<float>();
    safetyTemp = doc["safetyTemp"].as<int>();
    refreshRate = doc["refreshRate"].as<int>();
    timeOffset = doc["timeOffset"].as<int>();
    tempCalibration = doc["tempOffset"].as<float>();

    // Print extracted values
    Serial.println("  Extracted Parameters:");
    Serial.print("    onTemperature: "); Serial.println(onTemperature);
    Serial.print("    offTemperature: "); Serial.println(offTemperature);
    Serial.print("    tempControlRange: "); Serial.println(tempControlRange);
    Serial.print("    safetyTemp: "); Serial.println(safetyTemp);
    Serial.print("    refreshRate: "); Serial.println(refreshRate);
    Serial.print("    timeOffset: "); Serial.println(timeOffset);
    Serial.print("    tempOffset: "); Serial.println(tempCalibration);

    lcdCommand("rec parameters");
}

// Handle parameters received via MQTT
void handleTemperatureProgram(const String& jsonPayload) 
{
    // Create a JSON document (adjust size if needed)
    StaticJsonDocument<4096> doc;

    DeserializationError error = deserializeJson(doc, jsonPayload);
    if (error) 
    {
        Serial.print("JSON Parsing Failed: ");
        Serial.println(error.c_str());
        return;
    }

    const char* days[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
    
    for (int d = 0; d < 7; d++) 
    {
        Serial.print(days[d]);
        Serial.print(": ");

        if (!doc.containsKey(days[d])) {  // Ensure key exists
            Serial.println(" (Missing Key!)");
            continue;
        }

        JsonArray dayTemps = doc[days[d]];
        for (int h = 0; h < 24 && h < dayTemps.size(); h++) {  // Prevent overflow
            tempSchedule[d][h] = dayTemps[h];
            Serial.print(tempSchedule[d][h]);  
            Serial.print(" ");
        }
        Serial.println();  // Newline for readability
    }

    lcdCommand("rec tempSchedule");
    
    Serial.println("Updated temperature schedule received!");
}