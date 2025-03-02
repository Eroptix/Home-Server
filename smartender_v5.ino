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

#define WIFI_SSID "UPC8F21BEF"
#define WIFI_PASS "k7pp3aexkmQh"

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "smartender";
const char* currentSwVersion = "1.0.6";
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
unsigned long previousMillis1 = 0;
const long period1 = refreshRate * 1000;
unsigned long previousMillisMQTT = 0;               // MQTT reconnect timing
unsigned long previousMillisWiFi = 0;               // WiFi reconnect timing
unsigned long mqttReconnectInterval = 5000;   // Check MQTT every 5 seconds
unsigned long wifiReconnectInterval = 5000;   // Check WiFi every 5 seconds 
unsigned long wifiRetryMaxInterval = 30000;    // 30 seconds max
unsigned long mqttRetryMaxInterval = 60000;    // 60 seconds max 
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
bool buttonCheck = false;
int glassWeight = 240;
float dailyAmount = 0;

// Setting PWM properties
const int ledChannel = 1;
const int resolution = 8;
int dutyCycleLED = 250;
const int sleepDutyCycleLED = 10;
const int operationDutyCycleLED = 250;
int operationFreq = 5000;

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
String weight_topic =                       String("home/") + deviceName + String("/weight");
String dailyamount_topic =                  String("home/") + deviceName + String("/dailyamount");
String pump1_topic =                        String("home/") + deviceName + String("/pumps/pump1");
String pump2_topic =                        String("home/") + deviceName + String("/pumps/pump2");
String pump3_topic =                        String("home/") + deviceName + String("/pumps/pump3");
String pump4_topic =                        String("home/") + deviceName + String("/pumps/pump4");
String pump5_topic =                        String("home/") + deviceName + String("/pumps/pump5");
String pump6_topic =                        String("home/") + deviceName + String("/pumps/pump6");
String motor_topic =                        String("home/") + deviceName + String("/motor");
String fan_topic =                          String("home/") + deviceName + String("/fan");
String peltier_topic =                      String("home/") + deviceName + String("/peltier");

// Parameters
String button_topic =                       String("home/") + deviceName + String("/parameters/button");
String drink_topic =                        String("home/") + deviceName + String("/drink");
String glassweight_topic =                  String("home/") + deviceName + String("/parameters/glassweight");
String calibration_topic =                  String("home/") + deviceName + String("/calibration");

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
  else if (String(topic) == drink_topic) 
  {
    handleDrinks(message);
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

// Send discovery topics to Home Assistant
void sendDiscoveries()
{
  // Diagnostics
  publishMQTTDiscovery("Up Time", "sensor","mdi:clock", "h", "duration", "total_increasing", "diagnostic", uptime_topic);
  delay(200);
	publishMQTTDiscovery("OTA Status", "sensor", "mdi:update", "", "", "", "diagnostic", ota_status_topic);
  delay(200);
	publishMQTTDiscovery("Firmware Version", "sensor", "mdi:application-outline", "", "", "", "diagnostic", firmware_topic);
  delay(200);
	publishMQTTDiscovery("Error", "sensor", "mdi:alert-circle-outline", "", "", "", "diagnostic", log_error_topic);
  delay(200);
	publishMQTTDiscovery("Warning", "sensor", "mdi:shield-alert-outline", "", "", "", "diagnostic", log_warning_topic);
  delay(200);
	publishMQTTDiscovery("Info", "sensor", "mdi:information-outline", "", "", "", "diagnostic", log_info_topic);
  delay(200);
	publishMQTTDiscovery("IP Address", "sensor", "mdi:ip-network-outline", "", "", "", "diagnostic", ip_topic);
  delay(200);
  publishMQTTDiscovery("WiFi Strength", "sensor", "mdi-rss", "", "", "", "diagnostic", wifi_strength_topic);
  delay(200);
  // Sensors
  publishMQTTDiscovery("Weight", "sensor", "mdi:weight", "g", "weight", "measurement", "", weight_topic);
  delay(200);
  publishMQTTDiscovery("Daily Amount", "sensor", "mdi:cup", "L", "volume", "measurement", "", dailyamount_topic);
  delay(200);
  publishMQTTDiscovery("Pump 1", "binary_sensor", "mdi:pump", "", "moving", "", "", pump1_topic);
  delay(200);
  publishMQTTDiscovery("Pump 2", "binary_sensor", "mdi:pump", "", "moving", "", "", pump2_topic);
  delay(200);
  publishMQTTDiscovery("Pump 3", "binary_sensor", "mdi:pump", "", "moving", "", "", pump3_topic);
  delay(200);
  publishMQTTDiscovery("Pump 4", "binary_sensor", "mdi:pump", "", "moving", "", "", pump4_topic);
  delay(200);
  publishMQTTDiscovery("Pump 5", "binary_sensor", "mdi:pump", "", "moving", "", "", pump5_topic);
  delay(200);
  publishMQTTDiscovery("Pump 6", "binary_sensor", "mdi:pump", "", "moving", "", "", pump6_topic);
  delay(200);
  publishMQTTDiscovery("Fan", "binary_sensor", "mdi:fan", "", "running", "", "", fan_topic);
  delay(200);
  publishMQTTDiscovery("Motor", "binary_sensor", "mdi:rotate-360", "", "running", "", "", motor_topic);
  delay(200);
  publishMQTTDiscovery("Peltier", "binary_sensor", "mdi:snowflake-alert", "", "running", "", "", peltier_topic);
  delay(200);
  // Parameters
  publishMQTTDiscovery("Glass Weight", "sensor", "mdi:glass-cocktail", "g", "weight", "measurement", "diagnostic", glassweight_topic);
  delay(200);
  publishMQTTDiscovery("Calibration", "sensor", "mdi:calculator-variant-outline", "", "", "measurement", "diagnostic", calibration_topic);
  delay(200);
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

  // Initialize dashboard
  consoleLog("Initializing dashboard", 1);
  publishMessage(pump1_topic, pumpStatus[1] ? "ON" : "OFF", false);
  publishMessage(pump2_topic, pumpStatus[2] ? "ON" : "OFF", false);
  publishMessage(pump3_topic, pumpStatus[3] ? "ON" : "OFF", false);
  publishMessage(pump4_topic, pumpStatus[4] ? "ON" : "OFF", false);
  publishMessage(pump5_topic, pumpStatus[5] ? "ON" : "OFF", false);
  publishMessage(pump6_topic, pumpStatus[6] ? "ON" : "OFF", false);
  publishMessage(motor_topic, statusMotor ? "ON" : "OFF", false);
  publishMessage(fan_topic, statusFan ? "ON" : "OFF", false);
  publishMessage(peltier_topic, statusPeltier ? "ON" : "OFF", false);
  publishMessage(dailyamount_topic, dailyAmount, false);
  
  delay(2000);

  // attach the channel to the GPIO to be controlled
  ledcSetup(ledChannel, operationFreq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, dutyCycleLED);

  // Calibrate scale
  if(!debugMode){calibrateScale(glassWeight);}

  // Home Z axis
  if(!debugMode){stepperHoming();}

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
  
  client.loop();

  // Check button state
  checkPushButton();

  // Standby loop
  if (currentMillis - previousMillis1 >= period1) 
  { 
      previousMillis1 = currentMillis;
      
      standByWeight = readWeightSensor();

      if(standByWeight<100)
      {
        consoleLog("Place your glass on the stand", 1);
      }
      else 
      {
        consoleLog("Ready to serve drinks", 1);
      }
      
      // Check sleep time
      if(currentMillis - activityMillis > sleepTime)
      {
        // Sleep mode activated
        sleepModeON();
      }

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

    publishMessage(motor_topic, statusMotor ? "ON" : "OFF", false); 
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

    publishMessage(peltier_topic, statusPeltier ? "ON" : "OFF", false);
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

    publishMessage(fan_topic, statusFan ? "ON" : "OFF", false);
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
  
}

void sleepModeON()
{
    Serial.println("--------------------------------");
    Serial.println("        Sleep Mode ON");
    Serial.println("--------------------------------");

    consoleLog("Entering sleep mode", 1);
    
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

    consoleLog("Waking up from sleep mode", 1);
    
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
            
            // Publish manual drink trigger
            //publishMessage(button_topic, "true", false);

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
  if (message == "reboot") 
  {
    consoleLog("Command received: Reboot", 1);
    restartESP();
  }
  if (message == "request parameters") 
  {
    consoleLog("Command received: Parameter Request", 1);
    requestParameters();
  }
  if (message == "send parameters") 
  {
    consoleLog("Command received: Parameter Send", 1);
    sendParameters();
  }
  if (message == "send discoveries") 
  {
    consoleLog("Command received: Discovery Send", 1);
    sendDiscoveries();
  }
  if (message == "servo off") 
  {
    consoleLog("Command received: Servo Off", 1);
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
  if(totalVolume > 250)
  {
    consoleLog("Total requested drink volume exceeds maximum", 2);
    return;
  }

  // Start dispensing drink with the received recipe
  consoleLog(String("Starting mixing drink: ") + name, 1);

  if(head){stepper.runToNewPosition(headMovement);}       // Lower stirring head
  if(pump3 != 0){dispense(3, pump3);}                     // Dispense with pump3
  delay(1000);
  if(pump4 != 0){dispense(4, pump4);}                     // Dispense with pump4
  delay(1000);
  if(pump5 != 0){dispense(5, pump5);}                     // Dispense with pump5
  delay(1000);
  if(pump6 != 0){dispense(6, pump6);}                     // Dispense with pump6
  delay(1000);
  if(pump1 != 0){dispense(1, pump1);}                     // Dispense with pump1
  delay(1000);
  if(pump2 != 0){dispense(2, pump2);}                     // Dispense with pump2
  delay(1000);
  if(stirring != 0){motorStirring(stirring);}             // Stir the finished drink
  if(head){stepper.runToNewPosition(0);}                  // Raise stirring head

  consoleLog("Drink finished", 1);

  waitForRemove();
}