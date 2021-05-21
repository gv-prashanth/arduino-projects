/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank.
   Using 433MHz ASK RF Transmitter and Receiver modules for wireless link, along with RH_ASK to transmit the water level reading.
   Make sure you connect the RF Receiver to PIN 11 & RF Transmitter to PIN 12
   Arduino as logic controller to drive a sumpMotor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank is connected to a PWM based TOF distance sensor
   sumpMotorTriggerPin driver attached to pin 8 with 10k resistor to ground
*/

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

//Pin Configurations
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpDangerIndicatorPin = 6; // Sump danger level led pin
const int SumpReceiverIndicatorPin = 13; // Sump receiver indication led pin

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 10000; // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000; //in milliseconds
const unsigned long PROTECTION_FOR_DRY_RUN = 3600000; //in milliseconds
const unsigned long PROTECTION_TIME_FOR_RATE_CHECK = 300000; //in milliseconds
const int MAXIMUM_WATER_HEIGHT_ALLOWED = 110; // in centimeters
const int TANK_TOLERANCE = 10; // in centimeters
const int HEIGHT_OF_TOF_SENSOR_MEASURED_FROM_MAXIMUM_WATER_HEIGHT_ALLOWED = 5; // in centimeters

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime, lastSwitchOnTime, lastRateCheckTime;
float cached_overheadTankWaterLevel, lastRateCheckValue;
boolean firstTimeStarting, isMotorRunning;
RH_ASK driver;

void setup()
{
  // initialize serial communication with computer:
#ifdef RH_HAVE_SERIAL
  Serial.begin(9600);    // Debugging only
#endif
  if (!driver.init())
#ifdef RH_HAVE_SERIAL
    Serial.println("init failed");
#else
    ;
#endif

  // initialize the sumpMotorTriggerPin pin, SumpDangerIndicatorPin as Output
  pinMode(sumpMotorTriggerPin, OUTPUT);
  pinMode(SumpDangerIndicatorPin, OUTPUT);
  pinMode(SumpReceiverIndicatorPin, OUTPUT);

  // initialize all the necessary readings
  unsigned long currentTime = millis();
  lastSuccesfulOverheadTransmissionTime = currentTime;
  lastSwitchOnTime = currentTime;
  lastSwitchOffTime = currentTime;
  lastRateCheckTime = currentTime;
  firstTimeStarting = true;
  isMotorRunning = false;
}

void loop()
{
  // read values from all sensors
  loadAndCacheOverheadTransmissions();

  // display connection strength
  if (isConnectionWithinTreshold())
    digitalWrite(SumpReceiverIndicatorPin, HIGH);
  else
    digitalWrite(SumpReceiverIndicatorPin, LOW);

  if (isMotorRunning) {
    if (!isConnectionWithinTreshold()) {
      switchOffMotor();
      Serial.println("No strong signal from overhead. Switching OFF!");
    } else if (isMotorInDanger()) {
      switchOffMotor();
      Serial.println("Danger! Out of water in sump. Shutting down!");
      while (true)
        digitalWrite(SumpDangerIndicatorPin, HIGH);
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
      switchOnMotor();
      Serial.println("No water in overhead tank. Also sump has water. Switching ON!");
    } else if (!overheadBottomHasWater() && isRecentlySwitchedOff()) {
      Serial.println("No water in overhead tank. But very recently switched Off. Lets wait for protection time and then check.");
    } else {
      //Serial.println("Lets leave the motor in OFF.");
    }
  }
}

void switchOffMotor() {
  lastSwitchOffTime = millis();
  digitalWrite(sumpMotorTriggerPin, LOW);
  isMotorRunning = false;
}

void switchOnMotor() {
  if (firstTimeStarting)
    firstTimeStarting = false;
  lastSwitchOnTime = millis();
  digitalWrite(sumpMotorTriggerPin, HIGH);
  isMotorRunning = true;
  lastRateCheckTime = lastSwitchOnTime;
  lastRateCheckValue = cached_overheadTankWaterLevel;
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
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    int i;
    // Message with a good checksum received, dump it.
    //driver.printBuffer("Got:", buf, buflen);
    String respString = (char*)buf;
    cached_overheadTankWaterLevel = (MAXIMUM_WATER_HEIGHT_ALLOWED + HEIGHT_OF_TOF_SENSOR_MEASURED_FROM_MAXIMUM_WATER_HEIGHT_ALLOWED) - respString.toFloat();
    if (cached_overheadTankWaterLevel < 30)
      cached_overheadTankWaterLevel = 30;
    Serial.print("Tank water: "); Serial.print(cached_overheadTankWaterLevel); Serial.println(" cm");
    lastSuccesfulOverheadTransmissionTime = millis();
  }
}

boolean isMotorInDanger() {
  unsigned long currentTime = millis();
  boolean isMaxMotorRunTimeReached = currentTime - lastSwitchOnTime > PROTECTION_FOR_DRY_RUN;
  boolean isDryRunDetected = false;
//  if (currentTime - lastRateCheckTime > PROTECTION_TIME_FOR_RATE_CHECK) {
//    if (cached_overheadTankWaterLevel <= lastRateCheckValue)
//      isDryRunDetected = true;
//    lastRateCheckTime = currentTime;
//    lastRateCheckValue = cached_overheadTankWaterLevel;
//  }
  return isMaxMotorRunTimeReached || isDryRunDetected;
}

boolean isRecentlySwitchedOff() {
  return (!firstTimeStarting) && (millis() - lastSwitchOffTime < PROTECTION_BETWEEN_SWITCH_OFF_ON);
}

boolean overheadBottomHasWater() {
  return cached_overheadTankWaterLevel > (MAXIMUM_WATER_HEIGHT_ALLOWED - TANK_TOLERANCE);
}

boolean overheadTopHasWater() {
  return cached_overheadTankWaterLevel > MAXIMUM_WATER_HEIGHT_ALLOWED;
}
