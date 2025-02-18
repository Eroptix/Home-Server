#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* WIFI_SSID = "UPC8F21BEF";
const char* WIFI_PASS = "k7pp3aexkmQh";

// Device-specific settings
const char* deviceName = "testDevice";
const char* currentSwVersion = "1.3.0";
const char* deviceModel = "ESP32-C3";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";

String  firmwareUrl;
String  latestSwVersion;

IPAddress local_IP(192, 168, 0, 116);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

float testAttribute1;
float testAttribute2;

bool connectWifi()
{
  int attempts = 1;
  
  // Connect to WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.config(local_IP, gateway, subnet, dns);

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
  Serial.println("Rebooting ESP32");
  delay(2000);
  esp_restart();
}

/************************** HomeAssistant Settings ***********************************/

// MQTT broker details
const char* mqtt_server = "192.168.0.241";
const int mqtt_port = 1783;
const char* mqtt_client_id = deviceName;

// MQTT topics
const char* ota_query_topic = "home/ota/query";
const char* parameter_request_topic = "home/parameters/request";
String availability_topic = String("home/") + deviceName + String("/available");
String parameter_response_topic = String("home/") + deviceName + String("/parameters");
String ota_status_topic = String("home/") + deviceName + String("/ota/status");
String ota_response_topic = String("home/") + deviceName + String("/ota/response");
String log_info_topic = String("home/") + deviceName + String("/log/info");
String log_warning_topic = String("home/") + deviceName + String("/log/warning");
String log_error_topic = String("home/") + deviceName + String("/log/error");
String command_topic = String("home/") + deviceName + String("/command");
String uptime_topic = String("home/") + deviceName + String("/uptime");
String firmware_topic = String("home/") + deviceName + String("/firmware");
String ip_topic = String("home/") + deviceName + String("/ip");

// Initalize the Mqtt client instance
WiFiClient espClient;
PubSubClient client(espClient);

// Connect to the predefined MQTT broker
bool connectMQTT()
{
  bool connection = false;
  int attempts = 1;
  
  Serial.println("Connecting to MQTT server:");
  
  while (!client.connected() && attempts < 3) 
  {
    if ( client.connect(mqtt_client_id) ) {

      // Print MQTT topics used
      printMQTTTopics();

      // Subscribe to command topics
      subscribeTopic(ota_response_topic.c_str());
      subscribeTopic(parameter_response_topic.c_str());

      // Send discovery payload
	    sendDiscoveries();

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
}

// Handle Commands Received via MQTT
void handleCommand(String message) 
{
  if (message == "on") {
    Serial.println("Command received: ON");
    // Add functionality for "on" command
  } else if (message == "off") {
    Serial.println("Command received: OFF");
    // Add functionality for "off" command
  } else {
    Serial.print("Unknown command: ");
    Serial.println(message);
  }
}

// Handle Parameters Received via MQTT
void handleParameters(const String& jsonPayload) 
{
    // Create a JSON document (adjust size if needed)
    StaticJsonDocument<512> doc;

    // Deserialize the JSON
    DeserializationError error = deserializeJson(doc, jsonPayload);
    if (error) {
        Serial.print("JSON Parsing Failed: ");
        Serial.println(error.c_str());
        return;
    }

    // Extract values
    testAttribute1 = doc["testAttribute1"].as<float>();
    testAttribute2 = doc["testAttribute2"].as<float>();

    // Print extracted values
    Serial.println("  Extracted Parameters:");
    Serial.print("    testAttribute1: ");
    Serial.println(testAttribute1);
    Serial.print("    testAttribute2: ");
    Serial.println(testAttribute2);
}

// Subscribe to an MQTT topic
void subscribeTopic(const char* topic)
{
    client.subscribe(topic);
    Serial.print("	Subscribed to: ");
    Serial.println(topic);
}

// Publish an MQTT message with an int payload
void publishMessage(const char* topic, int payload, bool retain) 
{
  char message[16]; // Buffer to store the stringified payload
  itoa(payload, message, 10); // Convert int to string
  if (client.publish(topic, message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a double payload
void publishMessage(const char* topic, double payload, bool retain) 
{
  char message[16]; // Buffer to store the stringified payload
  dtostrf(payload, 1, 2, message); // Convert double to string with 2 decimal places
  if (client.publish(topic, message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a boolean payload
void publishMessage(const char* topic, bool payload, bool retain) 
{
  const char* message = payload ? "true" : "false"; // Convert bool to "true"/"false"
  if (client.publish(topic, message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a String payload
void publishMessage(const char* topic, String payload, bool retain) 
{
  const char* message = payload.c_str();
  if (client.publish(topic, message, retain)) {
    Serial.print("Published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message");
  }
}

// Publish an MQTT message with a const char* payload
void publishMessage(const char* topic, const char* payload, bool retain) 
{
  if (client.publish(topic, payload, retain)) {
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
  publishMessage(ota_status_topic.c_str(), "Updating", false);
  
  // Resource optimization: Check heap memory
  if (ESP.getFreeHeap() < 20000) {
    Serial.println("[ERROR] Not enough memory for OTA. Aborting.");
    publishMessage(ota_status_topic.c_str(), "Failed", false);
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
      publishMessage(ota_status_topic.c_str(), "Failed", false);
    }
  } else {
    Serial.printf("[ERROR] HTTP request failed with code: %d\n", httpCode);
    Serial.printf("[ERROR] HTTP error: %s\n", http.errorToString(httpCode).c_str());
    publishMessage(ota_status_topic.c_str(), "Failed", false);
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
        publishMessage(ota_status_topic.c_str(), "Updated", false);
        delay(10000);
        ESP.restart();
      } else {
        Serial.println("[ERROR] OTA update failed.");
        Update.printError(Serial);
        publishMessage(ota_status_topic.c_str(), "Failed", false);
      }
    } else {
      Serial.println("[ERROR] Not enough space for OTA update.");
      publishMessage(ota_status_topic.c_str(), "Failed", false);
    }
  } else {
    Serial.printf("[ERROR] HTTP request failed with code: %d\n", httpCode);
    publishMessage(ota_status_topic.c_str(), "Failed", false);
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
    performOTA();
  } else {
    Serial.println("[INFO] Device firmware is up to date.");
    publishMessage(ota_status_topic.c_str(), "Up to date", false);
  }
}

// Request server-side parameters from Home Assistant
void requestParameters()
{
  String requestPayload = String("{\"device\":\"") + deviceName + String("\"}");
  Serial.printf("[INFO] Publishing parameter request to MQTT: %s\n", requestPayload.c_str());
  client.publish(parameter_request_topic, requestPayload.c_str());
}

String removeSpaces(String input) {
  String output = "";
  for (int i = 0; i < input.length(); i++) {
    if (input[i] != ' ') {  
      output += input[i];  // Append only non-space characters
    }
  }
  return output;
}

// Create discovery payload
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
	publishMQTTDiscovery("Up Time", "sensor","mdi:clock", "h", "duration", "total_increasing", "diagnostic", uptime_topic);

	publishMQTTDiscovery("OTA Status", "sensor", "mdi:update", "", "", "", "diagnostic", ota_status_topic);

	publishMQTTDiscovery("Firmware Version", "sensor", "mdi:application-outline", "", "", "", "diagnostic", firmware_topic);

	publishMQTTDiscovery("Error", "sensor", "mdi:alert-circle-outline", "", "", "", "diagnostic", log_error_topic);

	publishMQTTDiscovery("Warning", "sensor", "mdi:shield-alert-outline", "", "", "", "diagnostic", log_warning_topic);

	publishMQTTDiscovery("Info", "sensor", "mdi:information-outline", "", "", "", "diagnostic", log_info_topic);

	publishMQTTDiscovery("IP Address", "sensor", "mdi:ip-network-outline", "", "", "", "diagnostic", ip_topic);
}

void printMQTTTopics()
{
  // Print used MQTT topics
  Serial.println("MQTT Topic:");
  Serial.println("    [availability_topic]: ");
  Serial.println(String("       home/") + deviceName + String("/availability"));
  Serial.println("    [ota_query_topic]: ");
  Serial.println("        home/ota/query");
  Serial.println("    [parameter_request_topic]: ");
  Serial.println("        home/parameters/request");
  Serial.println("    [parameter_response_topic]: ");
  Serial.println(String("       home/") + deviceName + String("/parameters"));
  Serial.println("    [ota_status_topic]");
  Serial.println(String("       home/") + deviceName + String("/ota/status"));
  Serial.println("    [ota_response_topic]");
  Serial.println(String("       home/") + deviceName + String("/ota/response"));
  Serial.println("    [log_info_topic]: ");
  Serial.println(String("       home/") + deviceName + String("/log/info"));
  Serial.println("    [log_warning_topic]");
  Serial.println(String("       home/") + deviceName + String("/log/warning"));
  Serial.println("    [log_error_topic]");
  Serial.println(String("       home/") + deviceName + String("/log/error"));
  Serial.println("    [command_topic]: ");
  Serial.println(String("       home/") + deviceName + String("/command"));
}

/************************** Setup ***********************************/

void setup() {
  delay(1000);  // Small delay before Serial.begin
  Serial.begin(115200);
  Serial.println("[INFO] Starting ESP32...");

  // Connect to Wi-Fi
  connectWifi();

  // Connect to MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(4096);
  connectMQTT();

  // Announce availability
  publishMessage(availability_topic.c_str(), "connected", false);

  // Query latest firmware version
  requestFirmwareVersion();
  delay(1000);

  // Request boot parameters
  requestParameters();
  delay(1000);

  // Set OTA progress callback
  Update.onProgress([](unsigned int progress, unsigned int total) 
  {
    Serial.printf("[INFO] OTA Progress: %u%%\r", (progress * 100) / total);
    Serial.println();
  });

  // Publish data to the server
  publishMessage(firmware_topic.c_str(), currentSwVersion, false);
  publishMessage(log_info_topic.c_str(), "Starting main loop", false);
  publishMessage(ip_topic.c_str(), configurationUrl, false);

}

/************************** Main Loop ***********************************/

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();
}