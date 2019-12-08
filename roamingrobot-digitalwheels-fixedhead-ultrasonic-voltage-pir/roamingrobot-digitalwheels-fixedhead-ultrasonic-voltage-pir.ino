#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DigitalBase.h>
#include <MorseCode.h>
#include <DeepSleep.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 11;//10
const int rightWheelBackwardPin = 10;//11
const int smartPowerPin = 6; //can be used to sleep and wake peripherals
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int batteryVoltageSensePin = -1;//A2 incase you want to detect from dedicated pin. -1 incase you want to detect from vcc.
const int pirInterruptPin = 2;//pin 2 only should be used

//functional Configuration
const int avoidableObstacleRange = 60;//cm
const int emergencyObstacleRange = avoidableObstacleRange / 3; //cm
const int calibratedMovementTime = 7500;//milli seconds
const int talkFrequency = 2000;//frequency in Hz
const int morseUnit = 200; //unit of morse
const unsigned long robotJamCheckTime = 180000; //milli seconds
const float sleepVoltage = 3.0;//volts
const float wakeVoltage = 3.6;//volts. Must be greater than sleepVoltage.
const int sleepCheckupTime = 300;//sec
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.

//Dont touch below stuff
VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
MorseCode morseCode(speakerPin, talkFrequency, morseUnit);
DeepSleep deepSleep;
unsigned long lastDirectionChangedTime = 0;
boolean isMarkedForSleep = false;
unsigned long sleepCounter = 0;
boolean isBatteryChargedWhileSleeping_Cached = false;
boolean isIntruderDetected = false;

void setup() {
  Serial.begin (9600);
  pinMode(pirInterruptPin, INPUT);// define interrupt pin D2 as input to read interrupt received by PIR sensor
  pinMode(smartPowerPin, OUTPUT);
  //TODO: Need to think of better place to write this logic
  digitalWrite(smartPowerPin, HIGH);

  //Wake the robot
  markForWakeup();

  //TODO: Need to improve this approach
  //Tell the voltage of battery
  float floatVoltage = batteryVoltageSensor.senseVoltage();
  morseCode.play(String(floatVoltage));

  //check if battery is low and skip bios dance
  if (!isBatteryDead()) {
    doBIOSManoeuvre();
  }

}

void loop() {
  //TODO: Need to think of better place to write this logic
  digitalWrite(smartPowerPin, HIGH);

  if (isMarkedForSleep) {

    //check if battery is charged and wake up
    if (isBatteryChargedWhileSleeping()) {
      markForWakeup();
      return;
    }

    //check if there was any intruder recently
    if (isIntruderDetected) {
      doIntruderManoeuvre();
      return;
    }

    //check if battery is low and continue sleep
    else {
      doSleepForEightSeconds();
      return;
    }

  } else {

    //check if battery is low and go to sleep
    if (isBatteryDead()) {
      markForSleep();
      return;
    }

    //check if there is a robot jam
    if (isJamDetected()) {
      doJamManoeuvre();
      return;
    }

    //incase of emergency stop
    if (isEmergencyObstaclePresent()) {
      doEmergencyObstacleManoeuvre();
      return;
    }

    //rotate left or right
    if (isAvoidableObstaclePresent()) {
      doAvoidableObstacleManoeuvre();
      return;
    }

    //go forward
    else {
      doStraightManoeuvre();
      return;
    }

  }

}

void markForSleep() {
  base.stop();
  morseCode.play("SOS");
  isMarkedForSleep = true;
  //TODO: Need to get rid of below
  lastDirectionChangedTime = 0;
}

void markForWakeup() {
  morseCode.play("Awake");
  isMarkedForSleep = false;
  //TODO: Need to get rid of below
  lastDirectionChangedTime = millis();
}

boolean isBatteryChargedWhileSleeping() {
  if (sleepCounter % (sleepCheckupTime / 8) == 0)
    isBatteryChargedWhileSleeping_Cached = batteryVoltageSensor.senseVoltage() > wakeVoltage;
  return isBatteryChargedWhileSleeping_Cached;
}

boolean isJamDetected() {
  return abs(millis() - lastDirectionChangedTime) > robotJamCheckTime;
}

boolean isBatteryDead() {
  return batteryVoltageSensor.senseVoltage() < sleepVoltage;
}

void intruderDetected() {
  isIntruderDetected = true;
}

boolean isAvoidableObstaclePresent() {
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
  return (centerReading > 0 && centerReading <= avoidableObstacleRange);
}

boolean isEmergencyObstaclePresent() {
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
  return (centerReading > 0 && centerReading <= emergencyObstacleRange);
}

void doStraightManoeuvre() {
  base.goForward();
}

void doEmergencyObstacleManoeuvre() {
  base.stop();
  tone(speakerPin, talkFrequency, 100);
  base.goBackward();
  delay(calibratedMovementTime / M_PI);
  if (decideOnRight())
    base.rotateRight();
  else
    base.rotateLeft();
  delay(random(9, 18) * 10 * (calibratedMovementTime / 360));
  base.stop();
  lastDirectionChangedTime = millis();
}

void doJamManoeuvre() {
  base.stop();
  morseCode.play("JAMMED");
  base.goBackward();
  delay(calibratedMovementTime / M_PI);
  if (decideOnRight())
    base.rotateRight();
  else
    base.rotateLeft();
  delay(random(0, 36) * 10 * (calibratedMovementTime / 360));
  base.stop();
  lastDirectionChangedTime = millis();
}

void doAvoidableObstacleManoeuvre() {
  if (decideOnRight())
    base.rotateRight();
  else
    base.rotateLeft();
  delay(random(0, 9) * 10 * (calibratedMovementTime / 360));
  base.stop();
  lastDirectionChangedTime = millis();
}

void doIntruderManoeuvre() {
  morseCode.play("INTRUDER");
  isIntruderDetected = false;
}

void doSleepForEightSeconds() {
  //Attach the PIR to activate intruder detection
  isIntruderDetected = false;
  attachInterrupt(digitalPinToInterrupt(pirInterruptPin), intruderDetected, RISING);

  deepSleep.sleepForEightSecondsUnlessInterrupted();

  //Detach the PIR since we dont need intruder detection anymore
  detachInterrupt(digitalPinToInterrupt(pirInterruptPin));

  sleepCounter++;
}

void doBIOSManoeuvre() {
  //left
  base.rotateLeft();
  delay(calibratedMovementTime);
  base.stop();
  morseCode.play("Left");

  //right
  base.rotateRight();
  delay(calibratedMovementTime);
  base.stop();
  morseCode.play("Right");

  //forward
  base.goForward();
  delay(calibratedMovementTime / M_PI);
  base.stop();
  morseCode.play("Forward");

  //backward
  base.goBackward();
  delay(calibratedMovementTime / M_PI);
  base.stop();
  morseCode.play("Backward");

  lastDirectionChangedTime = millis();
}

boolean decideOnRight() {
  int randNumber = random(0, 2);
  if (randNumber < 1) {
    return true;
  } else {
    return false;
  }
}
