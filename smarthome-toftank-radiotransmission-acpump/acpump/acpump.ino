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

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif
#include <LiquidCrystal.h>

//Pin Configurations
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpDangerIndicatorPin = 9; // Sump danger level led pin
const int SumpReceiverIndicatorPin = 13; // Sump receiver indication led pin

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 36000; // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000; //in milliseconds
const unsigned long MAX_ALLOWED_RUNTIME_OF_MOTOR = 1800000; //in milliseconds
const unsigned long BATCH_DURATION = 120000; //in milliseconds
const float HEIGHT_OF_TOF_SENSOR_FROM_GROUND = 115.0; // in centimeters
const float HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND = 108.0; // in centimeters
const float DIAMETER_OF_TANK = 108.0;//in centimers
const float TANK_TOLERANCE = 20.0; // in centimeters
const unsigned long DISPLAY_DURATION = 10000;                    //in milliseconds
const float VOLTAGE_TOLERANCE = 0.3; //volts
// initialize the library by associating any needed LCD interface pin with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime, lastSwitchOnTime, batchTimestamp, todayTracker_volume, todayTracker_time, todayTracker_switchOffHeight, batchCounter, displayChange;
float cached_overheadTankWaterLevel, cached_overheadTankWaterLevel_thisBatchAverage, cached_overheadTankWaterLevel_prevBatchAverage, cached_overheadTankWaterLevel_prevprevBatchAverage, cached_transmitterVcc, prevOverheadVoltage;
boolean firstTimeStarting, isMotorRunning, wasMotorInDangerInLastRun, prevLoopIsConnectionWithinTreshold, sendToAlexa;;
int prevTankPercentage, displayScreen;
RH_ASK driver;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
  // initialize serial communication with computer:
  Serial.begin(9600);    // Debugging only

  if (!driver.init())
    Serial.println("init failed");

  lcd.begin(16, 2);

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

void loop()
{
  // read values from all sensors
  loadAndCacheOverheadTransmissions();

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
  todayTracker_volume = calculateVolumeConsumedSoFar();//have to be first line since firstTimeStarting will get changed later
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
  }
  else {
    return (currentTime - lastSuccesfulOverheadTransmissionTime < TRANSMISSION_TRESHOLD_TIME);
  }
}

void loadAndCacheOverheadTransmissions() {
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  unsigned long currentTime = millis();
  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    int i;
    // Message with a good checksum received, dump it.
    //driver.printBuffer("Got:", buf, buflen);
    String fullString = (char*)buf;
    int index = fullString.indexOf(',');
    String respString = fullString.substring(0, index);
    String vccString = fullString.substring(index + 1);
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

    Serial.print("Tank water: "); Serial.print(cached_overheadTankWaterLevel); Serial.print(" cm. "); Serial.print("Prev: "); Serial.print(cached_overheadTankWaterLevel_prevBatchAverage); Serial.print(" cm. "); Serial.print("PrevPrev: "); Serial.print(cached_overheadTankWaterLevel_prevprevBatchAverage); Serial.println(" cm.");
  }

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
  if (isConnectionWithinTreshold() && displayScreen==1) {
    lcd.setCursor(0, 0);
    lcd.print(String("Capacity: ") + String(calculateTankPercentage()) + String("%          "));
    lcd.setCursor(0, 1);
    lcd.print(String("Usage: ") + String(calculateVolumeConsumedSoFar()) + String("L/") + String(calculateHoursConsumedSoFar()) + String("H          "));
  } else if(isConnectionWithinTreshold() && displayScreen==2) {
    lcd.setCursor(0, 0);
    lcd.print(String("Level: ") + String((int)cached_overheadTankWaterLevel) + String("cm          "));
    lcd.setCursor(0, 1);
    lcd.print(String("Battery: ") + String(cached_transmitterVcc) + String("v          "));
  } else {
    lcd.setCursor(0, 0);
    lcd.print(String("NO SIGNAL!!!"));
    lcd.setCursor(0, 1);
    lcd.print(String("Battery: ") + String(cached_transmitterVcc) + String("v          "));
  }
  unsigned long currentTime = millis();
  if(currentTime - displayChange > DISPLAY_DURATION){
    displayScreen++;
    if(displayScreen > 2)
      displayScreen = 1;
    displayChange = currentTime;
  }
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


void sendSensorValueToAlexa(String reading) {
  Serial.println("WaterTank " + reading);
}

void checkAndSendToAlexa() {
  if (prevTankPercentage != calculateTankPercentage()) {
    sendToAlexa = true;
    prevTankPercentage = calculateTankPercentage();
  }
  if (abs(prevOverheadVoltage - cached_transmitterVcc) > VOLTAGE_TOLERANCE) {
    sendToAlexa = true;
    prevOverheadVoltage = cached_transmitterVcc;
  }
  if (sendToAlexa) {
    Serial.println("sendToAlexa is invoked. Are you sure its worth invoking?");
    char destination[5];
    dtostrf(cached_transmitterVcc, 3, 1, destination);
    //String voltageInEncodedString = String((int)overheadVoltage);
    String voltageInEncodedString = String(destination);
    if (isMotorRunning) {
      if (wasMotorInDangerInLastRun) {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25%2E%20Motor%20is%20running%2E%20Voltage%20is%20" + voltageInEncodedString + "%20volts%2E%20There%20was%20dry%20run%2E");
      } else {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25%2E%20Motor%20is%20running%2E%20");
      }
    } else {
      if (wasMotorInDangerInLastRun) {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25%2E%20Motor%20is%20off%2E%20Voltage%20is%20" + voltageInEncodedString + "%20volts%2E%20There%20was%20dry%20run%2E");
      } else if (!isConnectionWithinTreshold()) {
        sendSensorValueToAlexa("not%20receiving%20from%20overhead%20tank%2E%20Last%20received%20Voltage%20is%20" + voltageInEncodedString + "%20volts%2E");
      } else {
        sendSensorValueToAlexa("at%20" + String(calculateTankPercentage()) + "%25%2E%20");
      }
    }
    sendToAlexa = false;
  }
}
