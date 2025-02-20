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

/************************** Device Settings ***********************************/

// Device-specific settings
const char* deviceName = "thermostat";
const char* currentSwVersion = "1.2.2";
const char* deviceModel = "ESP32-NodeMCU";
const char* deviceManufacturer = "BTM Engineering";
String configurationUrl = "";
String  firmwareUrl;
String  latestSwVersion;

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

// Setting PWM properties
const int ledChannel = 1;
const int resolution = 8;
int dutyCycleLED = 250;
const int sleepDutyCycleLED = 10;
const int operationDutyCycleLED = 250;
int operationFreq = 5000;

/************************** Device Functions ***********************************/

void consoleLog(const char* consoleText)
{
    //tb.sendTelemetryData("consoleLog", consoleText);
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
  Serial.println("Rebooting ESP32");
  delay(2000);
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

void setupBootParameters(String parameter, double value)
{

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

// Sensors
String temperature_topic =                  String("home/") + deviceName + String("/temperature");
String humidity_topic =                     String("home/") + deviceName + String("/humidity");
String pressure_topic =                     String("home/") + deviceName + String("/pressure");
String tempgoal_topic =                     String("home/") + deviceName + String("/tempgoal");
String mode_topic =                         String("home/") + deviceName + String("/mode");
String heating_topic =                      String("home/") + deviceName + String("/heating");

// Parameters
String ontemperature_topic =                String("home/") + deviceName + String("/parameters/ontemperature");
String offtemperature_topic =               String("home/") + deviceName + String("/parameters/offtemperature");
String tempcontrolrange_topic =             String("home/") + deviceName + String("/parameters/tempcontrolrange");
String safetytemp_topic =                   String("home/") + deviceName + String("/parameters/safetytemp");
String refreshrate_topic =                  String("home/") + deviceName + String("/parameters/refreshrate");
String timeoffset_topic =                   String("home/") + deviceName + String("/parameters/timeoffset");
String tempoffset_topic =                   String("home/") + deviceName + String("/parameters/tempoffset");

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
      subscribeTopic(ota_response_topic);
      subscribeTopic(command_topic);
      subscribeTopic(parameter_response_topic);

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
  if (String(topic) == climate_temperature_command_topic) 
  {
    handleTemperature(message);
  }
  if (String(topic) == climate_mode_command_topic) 
  {
    handleMode(message);
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
  publishMQTTDiscovery("Refresh Rate", "sensor", "mdi:refresh", "", "", "", "diagnostic", refreshrate_topic);
  publishMQTTDiscovery("Time Offset", "sensor", "mdi:clock-time-eight-outline", "", "", "", "diagnostic", timeoffset_topic);
  publishMQTTDiscovery("Temperature Offset", "sensor", "mdi:thermometer-chevron-up", "", "", "", "diagnostic", tempoffset_topic);
  // Climate
  publishMQTTClimateDiscovery("thermostat", "climate"); 
}

// Handle Commands Received via MQTT
void handleCommand(String message) 
{
  if (message == "reboot") 
  {
    Serial.println("Command received: Reboot");
    restartESP();
  }
  if (message == "request parameters") 
  {
    Serial.println("Command received: Parameter Request");
    requestParameters();
  }
  if (message == "send parameters") 
  {
    Serial.println("Command received: Parameter Send");
    sendParameters();
  }
  if (message == "send discoveries") 
  {
    Serial.println("Command received: Discovery Send");
    sendDiscoveries();
  }
  if (message == "servo off") 
  {
    Serial.println("Command received: Servo Off");
    servo.detach();
  }
}

// Handle Parameters Received via MQTT
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
    Serial.print("    onTemperature: ");
    Serial.println(onTemperature);
    Serial.print("    offTemperature: ");
    Serial.println(offTemperature);
    Serial.print("    tempControlRange: ");
    Serial.println(tempControlRange);
    Serial.print("    safetyTemp: ");
    Serial.println(safetyTemp);
    Serial.print("    refreshRate: ");
    Serial.println(refreshRate);
    Serial.print("    timeOffset: ");
    Serial.println(timeOffset);
    Serial.print("    tempOffset: ");
    Serial.println(tempCalibration);
}

// Send back received parameters to the server
void sendParameters()
{
  publishMessage(ontemperature_topic, onTemperature, true);
  publishMessage(offtemperature_topic, offTemperature, true);
  publishMessage(tempcontrolrange_topic, tempControlRange, true);
  publishMessage(safetytemp_topic, safetyTemp, true);
  publishMessage(refreshrate_topic, refreshRate, true);
  publishMessage(timeoffset_topic, timeOffset, true);
  publishMessage(tempoffset_topic, tempCalibration, true);
}

/************************** Setup function ***********************************/

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

/************************** Main loop ***********************************/

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
