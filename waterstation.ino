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

#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "waterstation";
const char* currentSwVersion = "1.0.1";
const char* deviceModel = "ESP32-NodeMCU";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";
String  firmwareUrl;
String  latestSwVersion;
bool debugMode = false;

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

// Sensors
String pump1_topic =                        String("home/") + deviceName + String("/pumps/pump1");
String pump2_topic =                        String("home/") + deviceName + String("/pumps/pump2");
String led_topic =                          String("home/") + deviceName + String("/led");
String moisture_meter_topic =               String("home/") + deviceName + String("/moisture");

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
}

// Send back received parameters to the server
void sendParameters()
{
  publishMessage(glassweight_topic, glassWeight, true);
}

/************************** Setup function ***********************************/

void setup(void)
{
  Serial.begin(115200);

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
  if (currentMillis - previousMillis1 >= period1) 
  { 
    previousMillis1 = currentMillis;
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
          publishMessage(pump1_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 2:
          publishMessage(pump2_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 3:
          publishMessage(pump3_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 4:
          publishMessage(pump4_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 5:
          publishMessage(pump5_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
          break;
       case 6:
          publishMessage(pump6_topic, pumpStatus[pumpID] ? "ON" : "OFF", false);
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
