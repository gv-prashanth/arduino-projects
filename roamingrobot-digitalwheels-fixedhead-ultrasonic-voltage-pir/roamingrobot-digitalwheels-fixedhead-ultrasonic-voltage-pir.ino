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
boolean isMarkedForSleep = false;
unsigned long sleepCounter = 0;
boolean isWakeVoltageReached_Cached = false;
boolean isMarkedForMovement = false;

void setup() {
  Serial.begin (9600);
  pinMode(pirInterruptPin, INPUT);        // define interrupt pin D2 as input to read interrupt received by PIR sensor

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

  //Wake the robot
  markForWakeup();

  //check if battery is low and skip bios dance
  if (!isBatteryLow()) {
    doBIOSManoeuvre();
  }

}

void loop() {

  if (isMarkedForSleep) {

    //check if battery is charged and wake up
    if (isWakeVoltageReached()) {
      markForWakeup();
      return;
    }

    //check if there was any movement recently
    if (isMarkedForMovement) {
      doMovementManoeuvre();
    }

    //check if battery is low and continue sleep
    else {
      SleepForEightSeconds();
      return;
    }

  } else {

    //check if battery is low and go to sleep
    if (isBatteryLow()) {
      markForSleep();
      return;
    }

    //check if there is a robot jam
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
    else {
      base.goForward();
      return;
    }

  }

}

void markForSleep() {
  base.stopAllMotion();
  morseCode.play("SOS");
  isMarkedForSleep = true;
}

void markForWakeup() {
  morseCode.play("Awake");
  isMarkedForSleep = false;
  lastEmergencyTime = millis() - 100; //just subtracting a small time
}

void markForMovement() {
  isMarkedForMovement = true;
}

void SleepForEightSeconds() {
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time

  //Attach the PIR to activate movement detection
  attachInterrupt(digitalPinToInterrupt(pirInterruptPin), markForMovement, RISING);
  isMarkedForMovement = false;

  //Begin the actual sleep
  __asm__  __volatile__("sleep");//in line assembler to go to sleep

  //Detach the PIR since we dont need movement detection anymore
  detachInterrupt(digitalPinToInterrupt(pirInterruptPin));

  sleepCounter++;
}

boolean isWakeVoltageReached() {
  if (sleepCounter % (sleepCheckupTime / 8) == 0)
    isWakeVoltageReached_Cached = voltageSensor.senseVoltage() > wakeVoltage;
  return isWakeVoltageReached_Cached;
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

void doMovementManoeuvre() {
  morseCode.play("ALARM");
  isMarkedForMovement = false;
}

void doBIOSManoeuvre() {
  //TODO: Need to improve this approach
  //Tell the voltage of battery
  float floatVoltage = voltageSensor.senseVoltage();
  morseCode.play(String(floatVoltage));

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
}//watchdog interrupt
