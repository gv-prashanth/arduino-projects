/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank.
   Using 433MHz ASK RF Transmitter and Receiver modules for wireless link, along with RH_ASK to transmit the water level reading.
   Make sure you connect the RF Receiver to PIN 11 & RF Transmitter to PIN 12 of Arduino
   Arduino as logic controller to drive a sumpMotor Pump. Pump is  connected through EM Relay mounted on Power supply unit.
   At Overhed tank Arduino along with RF Transmitter is connected to a PWM based TOF distance sensor OR Ultrasonic distance sensor
   sumpMotorTriggerPin driver attached to pin 8 with 10k resistor to ground
   SumpDangerIndicatorPin  9 connected with Red LED through 150E resistor to ground, indicating Dry Run
   SumpReceiverIndicatorPin  13 connected with Red LED through 150E resistor to ground indicating RF Link.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

#define WIFI_SSID "GTS"
#define WIFI_PASS "0607252609"

const String DROID_ID = "C3PO";

//Pin Configurations
const int sumpMotorTriggerPin = 14; // sump pump driver pin
const int SumpDangerIndicatorPin = 13; // Sump danger level led pin
const int SumpReceiverIndicatorPin = 12; // Sump receiver indication led pin

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 36000; // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000; //in milliseconds
const unsigned long MAX_ALLOWED_RUNTIME_OF_MOTOR = 3600000; //in milliseconds
const unsigned long BATCH_DURATION = 120000; //in milliseconds
const float HEIGHT_OF_TOF_SENSOR_FROM_GROUND = 118.0; // in centimeters
const float HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND = 108.0; // in centimeters
const float DIAMETER_OF_TANK = 108.0;//in centimers
const float TANK_TOLERANCE = 20.0; // in centimeters
char * ssid_ap = "ESP12E";
char * password_ap = "0607252609";
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime, lastSwitchOnTime, batchTimestamp, todayTracker_volume, todayTracker_time, todayTracker_switchOffHeight, batchCounter, lastSendCloudTime;
float cached_overheadTankWaterLevel, cached_overheadTankWaterLevel_thisBatchAverage, cached_overheadTankWaterLevel_prevBatchAverage, cached_overheadTankWaterLevel_prevprevBatchAverage;
boolean firstTimeStarting, isMotorRunning, wasMotorInDangerInLastRun;
IPAddress ip(192, 168, 11, 4); // arbitrary IP address (doesn't conflict w/ local network)
IPAddress gateway(192, 168, 11, 1);
IPAddress subnet(255, 255, 255, 0);
ESP8266WebServer server;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()
{
  // initialize serial communication with computer:
  Serial.begin(74880);    // Debugging only
  APwifiSetup();

  Wire.begin (4, 5);
  // Address 0x3C for 128x64, you might need to change this value (use an I2C scanner)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  Serial.println(F("display is success"));

  // initialize the sumpMotorTriggerPin pin, SumpDangerIndicatorPin as Output
  pinMode(sumpMotorTriggerPin, OUTPUT);
  pinMode(SumpDangerIndicatorPin, OUTPUT);
  pinMode(SumpReceiverIndicatorPin, OUTPUT);

  // initialize all the necessary readings
  unsigned long currentTime = millis();
  lastSuccesfulOverheadTransmissionTime = currentTime;
  lastSwitchOnTime = currentTime;
  lastSwitchOffTime = currentTime;
  batchTimestamp = currentTime;
  firstTimeStarting = true;
  isMotorRunning = false;
  wasMotorInDangerInLastRun = false;
  cached_overheadTankWaterLevel_prevBatchAverage = -1;
  cached_overheadTankWaterLevel_prevprevBatchAverage = -1;
  batchCounter = 0;

}

void loop()
{
  //receive any message from toftank
  server.handleClient();

  if (isMotorRunning) {
    if (!isConnectionWithinTreshold()) {
      switchOffMotor();
      Serial.println("No strong signal from overhead. Switching OFF!");
    } else if (isMotorInDanger()) {
      switchOffMotor();
      wasMotorInDangerInLastRun = true;
      Serial.println("Danger! Sump motor might be at risk. Shutting down!");
    } else if (overheadTopHasWater()) {
      switchOffMotor();
      Serial.println("Overhead tank is just filled. Switching OFF!");
    } else {
      //Serial.println("Lets leave the motor in ON.");
    }
  } else {
    if (!isConnectionWithinTreshold()) {
      Serial.println("No strong signal from overhead. Motor is already Switched OFF. Lets not switch ON!");
    } else if (!overheadBottomHasWater() && !isRecentlySwitchedOff()) {
      cached_overheadTankWaterLevel_prevBatchAverage = -1;
      cached_overheadTankWaterLevel_prevprevBatchAverage = -1;
      switchOnMotor();
      wasMotorInDangerInLastRun = false;
      Serial.println("No water in overhead tank. Also sump has water. Switching ON!");
    } else if (!overheadBottomHasWater() && isRecentlySwitchedOff()) {
      Serial.println("No water in overhead tank. But very recently switched Off. Lets wait for protection time and then check.");
    } else {
      //Serial.println("Lets leave the motor in OFF.");
    }
  }

  //do all displays
  displayLCDInfo();
  displayConnectionStrength();
  displaySumpDangerIndicator();
  turnOffAPTurnOnSTAndThenSendToCloud();
}

void displaySumpDangerIndicator() {
  if (wasMotorInDangerInLastRun)
    digitalWrite(SumpDangerIndicatorPin, HIGH);
  else
    digitalWrite(SumpDangerIndicatorPin, LOW);
}

void displayConnectionStrength() {
  if (isConnectionWithinTreshold())
    digitalWrite(SumpReceiverIndicatorPin, HIGH);
  else
    digitalWrite(SumpReceiverIndicatorPin, LOW);
}

void switchOffMotor() {
  lastSwitchOffTime = millis();
  digitalWrite(sumpMotorTriggerPin, LOW);
  isMotorRunning = false;
  todayTracker_switchOffHeight = cached_overheadTankWaterLevel;
}

void switchOnMotor() {
  todayTracker_volume = calculateVolumeConsumedSoFar();//have to be first line since firstTimeStarting will get changed later
  if (firstTimeStarting)
    firstTimeStarting = false;
  lastSwitchOnTime = millis();
  digitalWrite(sumpMotorTriggerPin, HIGH);
  isMotorRunning = true;
}

boolean isConnectionWithinTreshold() {
  unsigned long currentTime = millis();
  if (firstTimeStarting) {
    if (lastSuccesfulOverheadTransmissionTime > lastSwitchOnTime)
      return true;
    else
      return false;
  }
  else {
    return (currentTime - lastSuccesfulOverheadTransmissionTime < TRANSMISSION_TRESHOLD_TIME);
  }
}

void loadAndCacheOverheadTransmissions() {
  unsigned long currentTime = millis();

  cached_overheadTankWaterLevel = HEIGHT_OF_TOF_SENSOR_FROM_GROUND - server.arg("value").toFloat();
  server.send(200, "text/plain", "Updated");
  lastSuccesfulOverheadTransmissionTime = currentTime;

  cached_overheadTankWaterLevel_thisBatchAverage = ((batchCounter * cached_overheadTankWaterLevel_thisBatchAverage) / (batchCounter + 1)) + (cached_overheadTankWaterLevel / (batchCounter + 1));
  batchCounter++;
  if (currentTime - batchTimestamp > BATCH_DURATION) {
    batchTimestamp = currentTime;
    cached_overheadTankWaterLevel_prevprevBatchAverage = cached_overheadTankWaterLevel_prevBatchAverage;
    cached_overheadTankWaterLevel_prevBatchAverage = cached_overheadTankWaterLevel_thisBatchAverage;
    batchCounter = 0;
  }
  Serial.print("Tank water: "); Serial.print(cached_overheadTankWaterLevel); Serial.print(" cm. "); Serial.print("Prev: "); Serial.print(cached_overheadTankWaterLevel_prevBatchAverage); Serial.print(" cm. "); Serial.print("PrevPrev: "); Serial.print(cached_overheadTankWaterLevel_prevprevBatchAverage); Serial.println(" cm.");
}

boolean isMotorInDanger() {
  unsigned long currentTime = millis();
  if (!isMotorRunning)
    return false;
  else if (currentTime - lastSwitchOnTime > MAX_ALLOWED_RUNTIME_OF_MOTOR)
    return true; //Motor running for too long time. Its in danger.
  else if (cached_overheadTankWaterLevel_prevBatchAverage != -1 && cached_overheadTankWaterLevel_prevprevBatchAverage != -1 && cached_overheadTankWaterLevel_prevBatchAverage <= cached_overheadTankWaterLevel_prevprevBatchAverage)
    return true; //Motor is dry. Its in danger.
  else
    return false;
}

boolean isRecentlySwitchedOff() {
  return (!firstTimeStarting) && (millis() - lastSwitchOffTime < PROTECTION_BETWEEN_SWITCH_OFF_ON);
}

boolean overheadBottomHasWater() {
  return cached_overheadTankWaterLevel > (HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND - TANK_TOLERANCE);
}

boolean overheadTopHasWater() {
  return cached_overheadTankWaterLevel > HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND;
}

void displayLCDInfo() {
  // display Distance
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("LEVEL: ");
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print(String(calculateTankPercentage()) + String("% ") + String((int)cached_overheadTankWaterLevel) + String("cm"));
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("USAGE: ");
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(String(calculateVolumeConsumedSoFar()) + String("L/") + String(calculateHoursConsumedSoFar()) + String("H"));
  display.display();
}

int calculateTankPercentage() {
  return (cached_overheadTankWaterLevel / HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND) * 100;
}

int calculateHoursConsumedSoFar() {
  unsigned long currentTime = millis();
  int toReturn = (currentTime - todayTracker_time ) / (1000.0 * 60.0 * 60.0);
  if (toReturn >= 24) {
    todayTracker_volume = 0;
    todayTracker_time = currentTime;
  }
  return toReturn + 1;
}

unsigned long calculateVolumeConsumedSoFar() {
  if (isMotorRunning)
    return todayTracker_volume;
  else {
    if (todayTracker_switchOffHeight < cached_overheadTankWaterLevel)
      todayTracker_switchOffHeight = cached_overheadTankWaterLevel + 1;
    return todayTracker_volume + (2 * (todayTracker_switchOffHeight - cached_overheadTankWaterLevel) * PI * (DIAMETER_OF_TANK / 2) * (DIAMETER_OF_TANK / 2) * 0.001);
  }
}

void handleIndex() {
  server.send(200, "text/plain", String(cached_overheadTankWaterLevel)); // we'll need to refresh the page for getting the latest value
}

void STAwifiSetup() {
  WiFi.mode(WIFI_STA);
  Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  // Connected!
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void APwifiSetup(){
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(ssid_ap, password_ap);
  Serial.println();
  Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  // Configure the server's routes
  server.on("/", handleIndex); // use the top root path to report the last sensor value
  server.on("/update", loadAndCacheOverheadTransmissions); // use this route to update the sensor value
  server.begin();
}


void sendSensorValueToAlexa(String name, String reading) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String fullUrl = "https://home-automation.vadrin.com/droid/" + DROID_ID + "/upsert/intent/" + name + "/reading/" + reading;
  Serial.println("Requesting " + fullUrl);
  if (https.begin(client, fullUrl)) {
    int httpCode = https.GET();
    Serial.println("============== Response code: " + String(httpCode));
    if (httpCode > 0) {
      Serial.println(https.getString());
    }
    https.end();
  } else {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
}


void turnOffAPTurnOnSTAndThenSendToCloud(){
  unsigned long currentTime = millis();
  if(currentTime-lastSendCloudTime > 30000){
    Serial.printf("Sending to Cloud");
    STAwifiSetup();
    if(wasMotorInDangerInLastRun || !isConnectionWithinTreshold())
      sendSensorValueToAlexa("WaterTank", "in%20danger%2E%20Please%20check%20motor%2E");
    else{
      if(isMotorRunning)
        sendSensorValueToAlexa("WaterTank", "ON%2E%20It%20is%20at%20"+String(calculateTankPercentage())+"%25%2E%20"+String(calculateVolumeConsumedSoFar())+"%20liters%20is%20consumed%20in%20last%20"+String(calculateHoursConsumedSoFar())+"%20hours%2E");
      else
        sendSensorValueToAlexa("WaterTank", "OFF%2E%20It%20is%20at%20"+String(calculateTankPercentage())+"%25%2E%20"+String(calculateVolumeConsumedSoFar())+"%20liters%20is%20consumed%20in%20last%20"+String(calculateHoursConsumedSoFar())+"%20hours%2E");
    }
    APwifiSetup();
    lastSendCloudTime = currentTime;
    Serial.printf("Sent to Cloud");
  }
}