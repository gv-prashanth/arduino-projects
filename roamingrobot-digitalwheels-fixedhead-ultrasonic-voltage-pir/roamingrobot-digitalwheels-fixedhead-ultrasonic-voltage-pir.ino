#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DigitalBase.h>
#include <MorseCode.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 10;//10
const int rightWheelBackwardPin = 11;//11
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int voltagePin = A2;
const int pirInterruptPin = 2;//pin 2 only should be used

//functional Configuration
const int minimumRange = 25;//cm
const int calibratedMovementTime = 3500;//milli seconds
int robotWidth = 20;//cm
const int robotLength = 20;//cm
const int talkFrequency = 2000;//frequency in Hz
const int morseUnit = 300; //unit of morse
const int robotJamCheckTime = 30000; //milli seconds
const boolean rotateMode = true;
const float sleepVoltage = 3.5;//volts
const float wakeVoltage = 5.5;//volts. Must be greater than sleepVoltage.
const int sleepCheckupTime = 300;//sec

//Dont touch below stuff
VoltageSensor voltageSensor(voltagePin);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
MorseCode morseCode(speakerPin, talkFrequency, morseUnit);
unsigned long lastEmergencyTime = 0;
volatile boolean isMotionDetected = false;
boolean isSleeping = false;
unsigned long sleptTime = 0;

void setup() {
  Serial.begin (9600);
  attachInterrupt(digitalPinToInterrupt(pirInterruptPin), motionDetectedRoutine, RISING);

  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1 << 6); //enable interrupt mode

  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  //TODO: Quick hack to support both rotate and turn modes
  if (!rotateMode) {
    robotWidth = 2 * robotWidth;
  }

  // Wake the robot
  wakeUp();

  // check if battery is low and skip bios dance
  if (!isBatteryLow()) {
    doBIOSManoeuvre();
  }

}

void loop() {
  // check if battery is low and go to sleep
  if (!isSleeping && isBatteryLow()) {
    morseCode.play("SOS");
    goToSleep();
    return;
  }

  // check if battery is low and continue sleep
  if (isSleeping && !isMotionDetected && isFurtherSleepNeeded()) {
    SleepForEightSeconds();
    return;
  }

  // check if battery is charged and wake up
  if (isSleeping && !isMotionDetected && !isFurtherSleepNeeded()) {
    wakeUp();
    return;
  }

  // check if there is motion while sleeping
  if (isSleeping && isMotionDetected) {
    wakeUp();
    doMotionDetectManoeuvre();
    goToSleep();
    return;
  }

  // check if there is a robot jam
  if (isJamDetected()) {
    doEmergencyManoeuvre(10 * random(0, 36));
    return;
  }

  //check if there is an obstacle
  if (isObstaclePresent()) {
    tone(speakerPin, talkFrequency, 100);
    doEmergencyManoeuvre(90);
    return;
  }

  //go forward
  base.goForward();
}

void goToSleep() {
  base.stopAllMotion();
  //stabilizePIR
  delay(30000);
  isSleeping = true;
  isMotionDetected = false;
  sleptTime = 0;
  SleepForEightSeconds();
}

void wakeUp() {
  morseCode.play("Awake");
  isSleeping = false;
  lastEmergencyTime = millis() - 100; //just subtracting a small time
}

boolean isFurtherSleepNeeded() {
  //check battery every sleepCheckupTime seconds
  if (sleptTime > sleepCheckupTime && isWakeVoltageReached()) {
    return false;
  } else {
    return true;
  }
}

void SleepForEightSeconds() {
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  __asm__  __volatile__("sleep");//in line assembler to go to sleep
  sleptTime += 8;
}

boolean isWakeVoltageReached() {
  return voltageSensor.senseVoltage() > wakeVoltage;
}

boolean isJamDetected() {
  return abs(millis() - lastEmergencyTime) > robotJamCheckTime;
}

boolean isBatteryLow() {
  return voltageSensor.senseVoltage() < sleepVoltage;
}

boolean isObstaclePresent() {
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
  return (centerReading > 0 && centerReading <= minimumRange);
}

void motionDetectedRoutine() {
  isMotionDetected = true;
}

void doMotionDetectManoeuvre() {
  base.rotateRight((calibratedMovementTime / 360) * (180));
  morseCode.play("Danger");
  isMotionDetected = false;
}

void doEmergencyManoeuvre(int angle) {
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
  lastEmergencyTime = millis() - 100;
}

void doBIOSManoeuvre() {
  //Tell the voltage of battery
  int intVoltage = voltageSensor.senseVoltage();
  morseCode.play(String(intVoltage));

  //left
  if (rotateMode) {
    base.rotateLeft(calibratedMovementTime);
  } else {
    base.turnLeft(calibratedMovementTime);
  }
  morseCode.play("Left");

  //right
  if (rotateMode) {
    base.rotateRight(calibratedMovementTime);
  } else {
    base.turnRight(calibratedMovementTime);
  }
  morseCode.play("Right");

  //forward
  base.goForward();
  delay((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  base.stopAllMotion();
  morseCode.play("Forward");

  //backward
  base.moveBackward((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  morseCode.play("Backward");
}

boolean decideOnRight() {
  int randNumber = random(0, 2);
  if (randNumber < 1) {
    return true;
  } else {
    return false;
  }
}

ISR(WDT_vect) {
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}// watchdog interrupt
