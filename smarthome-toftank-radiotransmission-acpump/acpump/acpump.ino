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
#include <LiquidCrystal.h>

//Pin Configurations
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpDangerIndicatorPin = 6; // Sump danger level led pin
const int SumpReceiverIndicatorPin = 13; // Sump receiver indication led pin

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 36000; // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000; //in milliseconds
const unsigned long MAX_ALLOWED_RUNTIME_OF_MOTOR = 3600000; //in milliseconds
const unsigned long PROTECTION_TIME_FOR_RATE_CHECK = 120000; //in milliseconds
const float HEIGHT_OF_TOF_SENSOR_FROM_GROUND = 118.0; // in centimeters
const float HEIGHT_OF_TANK_DRAIN_OUT_FROM_GROUND = 108.0; // in centimeters
const float DIAMETER_OF_TANK = 108.0;//in centimers
const float TANK_TOLERANCE = 20.0; // in centimeters
// initialize the library by associating any needed LCD interface pin with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime, lastSwitchOnTime, rateCheck_lastTimestamp, todayTracker_volume, todayTracker_time, todayTracker_switchOffHeight;
float cached_overheadTankWaterLevel, rateCheck_lastValue;
boolean firstTimeStarting, isMotorRunning;
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
  rateCheck_lastTimestamp = currentTime;
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

  //do all displays
  displayLCDInfo();
  displayConnectionStrength();
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
  rateCheck_lastTimestamp = lastSwitchOnTime;
  rateCheck_lastValue = cached_overheadTankWaterLevel;
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
    cached_overheadTankWaterLevel = HEIGHT_OF_TOF_SENSOR_FROM_GROUND - respString.toFloat();
    Serial.print("Tank water: "); Serial.print(cached_overheadTankWaterLevel); Serial.println(" cm");
    lastSuccesfulOverheadTransmissionTime = millis();
  }
}

boolean isMotorInDanger() {
  unsigned long currentTime = millis();
  boolean isMaxMotorRunTimeReached = currentTime - lastSwitchOnTime > MAX_ALLOWED_RUNTIME_OF_MOTOR;
  boolean isDryRunDetected = false;
  if (currentTime - rateCheck_lastTimestamp > PROTECTION_TIME_FOR_RATE_CHECK) {
    if (cached_overheadTankWaterLevel <= rateCheck_lastValue)
      isDryRunDetected = true;
    rateCheck_lastTimestamp = currentTime;
    rateCheck_lastValue = cached_overheadTankWaterLevel;
  }
  return isMaxMotorRunTimeReached || isDryRunDetected;
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
  lcd.setCursor(0, 0); lcd.print(String("Lvl: ") + String(calculateTankPercentage()) + String("%(") + String((int)cached_overheadTankWaterLevel) + String("cm)          "));
  lcd.setCursor(0, 1); lcd.print(String("Use: ") + String(calculateVolumeConsumedSoFar()) + String("L/") + String(calculateHoursConsumedSoFar()) + String("H            "));
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
      todayTracker_switchOffHeight = cached_overheadTankWaterLevel+1;
    return todayTracker_volume + (2 * (todayTracker_switchOffHeight - cached_overheadTankWaterLevel) * PI * (DIAMETER_OF_TANK / 2) * (DIAMETER_OF_TANK / 2) * 0.001);
  }
}
