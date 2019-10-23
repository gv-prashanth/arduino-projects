#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DigitalBase.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 10;//10
const int rightWheelBackwardPin = 11;//11
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int voltagePin = A2;

//functional Configuration
const int minimumRange = 25;//cm
const int calibratedMovementTime = 3500;//milli seconds
int robotWidth = 20;//cm
const int robotLength = 20;//cm
const int talkFrequency = 2000;//frequency in Hz
const int shoutFrequency = 5000;//frquency in Hz
const int robotJamCheckTime = 15000; //milli seconds
const boolean rotateMode = true;
const float sleepCutoffVoltage = 3.5;

//Dont touch below stuff
VoltageSensor voltageSensor(voltagePin);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
unsigned long lastEmergencyTime = 0;

void setup() {
  //TODO: Quick hack to support both rotate and turn modes
  if (!rotateMode) {
    robotWidth = 2 * robotWidth;
  }
  checkBatteryVoltage();
  checkBaseHeadDirections();
  lastEmergencyTime = millis() - 100; //just subtracting a small time
}

void loop() {
  //go left or right
  int centerReading = (int) getReading();
  if (centerReading > 0 && centerReading <= minimumRange) {
    obstacleTooCloseEmergencyStop(90);
    return;
  }
  if (checkForJam()) {
    obstacleTooCloseEmergencyStop(10 * random(0, 36));
    return;
  }

  //go forward
  base.goForward();

  //go sleep
  float floatVoltage = voltageSensor.senseVoltage();
  if (floatVoltage < sleepCutoffVoltage) {
    //go to sleep for 30 Minuntes or so
    goToSleep();
  }
}

void goToSleep() {
  checkBatteryVoltage();
  //Delay wont actually sleep arduino. It will just sleep the motors. Fine for now.
  delay(60000);
}

boolean checkForJam() {
  if (abs(millis() - lastEmergencyTime) > robotJamCheckTime) {
    tone(speakerPin, talkFrequency, 500);
    return true;
  }
  return false;
}

void obstacleTooCloseEmergencyStop(int angle) {
  tone(speakerPin, shoutFrequency, 100);
  lastEmergencyTime = millis();
  base.moveBackward((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  if (decideOnRight()) {
    if (rotateMode) {
      base.rotateRight((calibratedMovementTime / 360)*angle);
    } else {
      base.turnRight((calibratedMovementTime / 360)*angle);
    }
  } else {
    if (rotateMode) {
      base.rotateLeft((calibratedMovementTime / 360)*angle);
    } else {
      base.turnLeft((calibratedMovementTime / 360)*angle);
    }
  }
}

int getReading() {
  int reading = (int) ultrasonicSensor.obstacleDistance();
  return reading;
}

void checkBaseHeadDirections() {
  if (rotateMode) {
    base.rotateLeft(calibratedMovementTime);
  } else {
    base.turnLeft(calibratedMovementTime);
  }
  delay(400);
  if (rotateMode) {
    base.rotateRight(calibratedMovementTime);
  } else {
    base.turnRight(calibratedMovementTime);
  }
  delay(400);
  base.goForward();
  delay((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  base.stopAllMotion();
  delay(400);
  base.moveBackward((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  delay(400);
}

boolean decideOnRight() {
  int randNumber = random(0, 2);
  if (randNumber < 1) {
    return true;
  } else {
    return false;
  }
}

void checkBatteryVoltage() {
  int intVoltage = voltageSensor.senseVoltage();
  for (int i = 0; i < intVoltage; i++) {
    tone(speakerPin, shoutFrequency, 200);
    delay(400);
  }
}
