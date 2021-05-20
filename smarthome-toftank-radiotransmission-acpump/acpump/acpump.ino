/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank.
   Using 433MHz RF Transmitter and Receiver modules for wireless link, along with RH_ASK to transmit the water level reading.
   Make sure you connect the RF Receiver to PIN 11 & RF Transmitter to PIN 12
   Arduino as logic controller to drive a sumpMotor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank is connected to a PWM based TOF distance sensor
   sumpMotorTriggerPin driver attached to pin 5  with 10k resistor to ground
*/

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

//Pin Configurations
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpDangerIndicatorPin = 11; // Sump danger level led pin

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 10000; // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000; //in milliseconds
const unsigned long PROTECTION_FOR_DRY_RUN = 3600000; //in milliseconds
const int MAXIMUM_WATER_HEIGHT_ALLOWED = 110; // in centimeters
const int TANK_TOLERANCE = 30; // in centimeters
const int HEIGHT_OF_TOF_SENSOR_MEASURED_FROM_MAXIMUM_WATER_HEIGHT_ALLOWED = 5; // in centimeters

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime, lastSwitchOnTime;
float cached_overheadTankWaterLevel;
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

  // initialize all the necessary readings
  unsigned long currentTime = millis();
  lastSuccesfulOverheadTransmissionTime = currentTime;
  lastSwitchOnTime = currentTime;
  lastSwitchOffTime = currentTime;
  firstTimeStarting = true;
  isMotorRunning = false;
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
    if (!overheadBottomHasWater() && !isRecentlySwitchedOff()) {
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
}

boolean isConnectionWithinTreshold() {
  unsigned long currentTime = millis();
  return currentTime - lastSuccesfulOverheadTransmissionTime < TRANSMISSION_TRESHOLD_TIME ;
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
    Serial.print("Tank water: "); Serial.print(cached_overheadTankWaterLevel); Serial.println(" cm");
    lastSuccesfulOverheadTransmissionTime = millis();
  }
}

boolean isMotorInDanger() {
  return millis() - lastSwitchOnTime > PROTECTION_FOR_DRY_RUN;
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
