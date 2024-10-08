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

#define TRANSMISSION_TYPE RH  // RH, LORA - Change PINS Below
#define DISPLAY_TYPE LCD  // NONE, LCD, OLED - Change PINS Below
const String TANK_NAME = "GTSPureWaterTank";

//Pin Configurations
const int sumpMotorTriggerPin = 8;        // sump pump driver pin. 8 for Arduino, 3 for ESP
const int SumpDangerIndicatorPin = 9;     // Sump danger level led pin. 9 for Arduino, 0 for ESP
const int SumpReceiverIndicatorPin = 13;  // Sump receiver indication led pin. 13 for Arduino, 2 for ESP

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 36000;          // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000;  //in milliseconds
const unsigned long MAX_ALLOWED_RUNTIME_OF_MOTOR = 1800000;      //in milliseconds
const unsigned long BATCH_DURATION = 120000;                     //in milliseconds
const float HEIGHT_OF_TOF_SENSOR_FROM_GROUND = 115.0;            // in centimeters
const float HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND = 108.0;        // in centimeters
const float DIAMETER_OF_TANK = 108.0;                            //in centimers
const float TANK_TOLERANCE = 20.0;                               // in centimeters
const unsigned long DISPLAY_DURATION = 10000;                    //in milliseconds
const float VOLTAGE_TOLERANCE = 0.1;                             //volts


//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime, lastSwitchOnTime, batchTimestamp, todayTracker_volume, todayTracker_time, todayTracker_switchOffHeight, batchCounter, displayChange;
float cached_overheadTankWaterLevel, cached_overheadTankWaterLevel_thisBatchAverage, cached_overheadTankWaterLevel_prevBatchAverage, cached_overheadTankWaterLevel_prevprevBatchAverage, cached_transmitterVcc, prevOverheadVoltage;
boolean firstTimeStarting, isMotorRunning, wasMotorInDangerInLastRun, prevLoopIsConnectionWithinTreshold, sendToAlexa;
int prevTankPercentage, displayScreen;
int calculateTankPercentage();
boolean isConnectionWithinTreshold();
unsigned long calculateVolumeConsumedSoFar();
int calculateHoursConsumedSoFar();
void loadAndCacheOverheadTransmissions(String fullString);
int signalStrength;

#define RH 1
#define LORA 2
#ifdef TRANSMISSION_TYPE
#if TRANSMISSION_TYPE == RH
#include "rh.h"                  // Include and use the RH_ASK display library
#elif TRANSMISSION_TYPE == LORA  // Include and use the LoRa display library
#include "lor.h"
#else
#error "Invalid library selection."
#endif
#else
#error "Library selection not defined."
#endif

#define NONE 1
#define LCD 2
#define OLED 3
#ifdef DISPLAY_TYPE
#if DISPLAY_TYPE == NONE
#include "none.h"          // Include and use the none display library
#elif DISPLAY_TYPE == LCD  // Include and use the Lcd display library
#include "lcd.h"
#elif DISPLAY_TYPE == OLED  // Include and use the oled display library
#include "oled.h"
#else
#error "Invalid library selection."
#endif
#else
#error "Library selection not defined."
#endif

void setup() {
  // initialize serial communication with computer:
  //DONT DISABLE.
  Serial.begin(9600);  // Mandatory if you want to use alexa integration.

  setupTransmission();

  setupDisplay();

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
  sendToAlexa = true;
  isMotorRunning = false;
  wasMotorInDangerInLastRun = false;
  prevLoopIsConnectionWithinTreshold = false;
  cached_overheadTankWaterLevel_prevBatchAverage = -1;
  cached_overheadTankWaterLevel_prevprevBatchAverage = -1;
  batchCounter = 0;
  displayScreen = 1;
}

void loop() {
  // read values from all sensors
  fetchOverheadReading();

  if (isMotorRunning) {
    if (!isConnectionWithinTreshold()) {
      switchOffMotor();
      //Serial.println("No strong signal from overhead. Switching OFF!");
    } else if (isMotorInDanger()) {
      switchOffMotor();
      wasMotorInDangerInLastRun = true;
      //Serial.println("Danger! Sump motor might be at risk. Shutting down!");
    } else if (overheadTopHasWater()) {
      switchOffMotor();
      //Serial.println("Overhead tank is just filled. Switching OFF!");
    } else {
      //Serial.println("Lets leave the motor in ON.");
    }
  } else {
    if (!isConnectionWithinTreshold()) {
      //Serial.println("No strong signal from overhead. Motor is already Switched OFF. Lets not switch ON!");
    } else if (!overheadBottomHasWater() && !isRecentlySwitchedOff()) {
      cached_overheadTankWaterLevel_prevBatchAverage = -1;
      cached_overheadTankWaterLevel_prevprevBatchAverage = -1;
      switchOnMotor();
      wasMotorInDangerInLastRun = false;
      //Serial.println("No water in overhead tank. Also sump has water. Switching ON!");
    } else if (!overheadBottomHasWater() && isRecentlySwitchedOff()) {
      //Serial.println("No water in overhead tank. But very recently switched Off. Lets wait for protection time and then check.");
    } else {
      //Serial.println("Lets leave the motor in OFF.");
    }
  }

  //do all displays
  displayScreenInfo();
  trackAndDisplayConnectionStrength();
  displaySumpDangerIndicator();
  checkAndSendToAlexa();
}

void displaySumpDangerIndicator() {
  if (wasMotorInDangerInLastRun)
    digitalWrite(SumpDangerIndicatorPin, HIGH);
  else
    digitalWrite(SumpDangerIndicatorPin, LOW);
}

void trackAndDisplayConnectionStrength() {
  unsigned long currentLoopIsConnectionWithinTreshold = isConnectionWithinTreshold();
  if (currentLoopIsConnectionWithinTreshold) {
    digitalWrite(SumpReceiverIndicatorPin, HIGH);
    if (!prevLoopIsConnectionWithinTreshold) {
      //we just got the signal
      unsigned long currentTime = millis();
      sendToAlexa = true;
    }
  } else {
    digitalWrite(SumpReceiverIndicatorPin, LOW);
    if (prevLoopIsConnectionWithinTreshold) {
      //we just lost the signal
      sendToAlexa = true;
    }
  }
  prevLoopIsConnectionWithinTreshold = currentLoopIsConnectionWithinTreshold;
}

void switchOffMotor() {
  lastSwitchOffTime = millis();
  digitalWrite(sumpMotorTriggerPin, LOW);
  isMotorRunning = false;
  todayTracker_switchOffHeight = cached_overheadTankWaterLevel;
  sendToAlexa = true;
}

void switchOnMotor() {
  todayTracker_volume = calculateVolumeConsumedSoFar();  //have to be first line since firstTimeStarting will get changed later
  if (firstTimeStarting)
    firstTimeStarting = false;
  lastSwitchOnTime = millis();
  digitalWrite(sumpMotorTriggerPin, HIGH);
  isMotorRunning = true;
  sendToAlexa = true;
}

boolean isConnectionWithinTreshold() {
  unsigned long currentTime = millis();
  if (firstTimeStarting && (currentTime - lastSwitchOnTime < TRANSMISSION_TRESHOLD_TIME)) {
    return false;
  } else {
    return (currentTime - lastSuccesfulOverheadTransmissionTime < TRANSMISSION_TRESHOLD_TIME);
  }
}

boolean isMotorInDanger() {
  unsigned long currentTime = millis();
  if (!isMotorRunning)
    return false;
  else if (currentTime - lastSwitchOnTime > MAX_ALLOWED_RUNTIME_OF_MOTOR)
    return true;  //Motor running for too long time. Its in danger.
  else if (cached_overheadTankWaterLevel_prevBatchAverage != -1 && cached_overheadTankWaterLevel_prevprevBatchAverage != -1 && cached_overheadTankWaterLevel_prevBatchAverage <= cached_overheadTankWaterLevel_prevprevBatchAverage)
    return true;  //Motor is dry. Its in danger.
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

int calculateTankPercentage() {
  return (cached_overheadTankWaterLevel / HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND) * 100;
}

int calculateHoursConsumedSoFar() {
  unsigned long currentTime = millis();
  int toReturn = (currentTime - todayTracker_time) / (1000.0 * 60.0 * 60.0);
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


void sendSensorValueToAlexa(String reading) {
  Serial.println("WaterTank " + reading);  // Mandatory if you want to use alexa integration.
}

void checkAndSendToAlexa() {
  if (abs(prevTankPercentage - ((int)calculateTankPercentage())) >= 1) {
    sendToAlexa = true;
    prevTankPercentage = (int)calculateTankPercentage();
  }
  if (abs(prevOverheadVoltage - cached_transmitterVcc) >= VOLTAGE_TOLERANCE) {
    sendToAlexa = true;
    prevOverheadVoltage = cached_transmitterVcc;
  }
  if (sendToAlexa) {
    //Serial.println("sendToAlexa is invoked. Are you sure its worth invoking?");
    char destination[5];
    dtostrf(cached_transmitterVcc, 3, 1, destination);
    //String voltageInEncodedString = String((int)overheadVoltage);
    String voltageInEncodedString = String(destination);
    if (isMotorRunning) {
      if (wasMotorInDangerInLastRun) {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25%2E%20Motor%20ON%2E%20Voltage%20" + voltageInEncodedString + "%20volts%2E%20DRY%20RUN%2E");
      } else {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25%2E%20Motor%20ON%2E");
      }
    } else {
      if (wasMotorInDangerInLastRun) {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25%2E%20Motor%20OFF%2E%20Voltage%20" + voltageInEncodedString + "%20volts%2E%20DRY%20RUN%2E");
      } else if (!isConnectionWithinTreshold()) {
        sendSensorValueToAlexa("not%20receiving%2E%20Last%20Voltage%20" + voltageInEncodedString + "%20volts%2E");
      } else {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25");
      }
    }
    sendToAlexa = false;
  }
}

void loadAndCacheOverheadTransmissions(String fullString) {
  unsigned long currentTime = millis();
  int index = fullString.indexOf(',');
  String respString = fullString.substring(0, index);
  String vccTankString = fullString.substring(index + 1);
  index = vccTankString.indexOf(',');
  String vccString = vccTankString.substring(0, index);
  String tankString = vccTankString.substring(index + 1);
  //Check for tank String and then resume
  if (!tankString.startsWith(TANK_NAME))
    return;
  //Serial.println("respString: " + respString);
  //Serial.println("vccString: " + vccString);
  //Serial.println("tankString: " + tankString);
  //Serial.println("Received Message from " + tankString + ". And will be processed since receiver is configured to " + TANK_NAME);
  //sometimes we are getting zero vcc from above. in such case, we used the cached value rather than setting it to zero in our receiver variable.
  if (vccString.toFloat() > 0)
    cached_transmitterVcc = vccString.toFloat() / 100;
  cached_overheadTankWaterLevel = HEIGHT_OF_TOF_SENSOR_FROM_GROUND - respString.toFloat();
  lastSuccesfulOverheadTransmissionTime = currentTime;

  cached_overheadTankWaterLevel_thisBatchAverage = ((batchCounter * cached_overheadTankWaterLevel_thisBatchAverage) / (batchCounter + 1)) + (cached_overheadTankWaterLevel / (batchCounter + 1));
  batchCounter++;
  if (currentTime - batchTimestamp > BATCH_DURATION) {
    batchTimestamp = currentTime;
    cached_overheadTankWaterLevel_prevprevBatchAverage = cached_overheadTankWaterLevel_prevBatchAverage;
    cached_overheadTankWaterLevel_prevBatchAverage = cached_overheadTankWaterLevel_thisBatchAverage;
    batchCounter = 0;
  }
  /*
    Serial.print("Tank water: ");
    Serial.print(cached_overheadTankWaterLevel);
    Serial.print(" cm. ");
    Serial.print("Prev: ");
    Serial.print(cached_overheadTankWaterLevel_prevBatchAverage);
    Serial.print(" cm. ");
    Serial.print("PrevPrev: ");
    Serial.print(cached_overheadTankWaterLevel_prevprevBatchAverage);
    Serial.println(" cm.");
  */  
}