#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DigitalBase.h>
#include <MorseCode.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 11;//10
const int rightWheelBackwardPin = 10;//11
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int batteryVoltageSensePin = A2;
const int solarVoltageSensePin = A0;
const int pirInterruptPin = 2;//pin 2 only should be used

//functional Configuration
const int minimumRange = 60;//cm
const int emergencyMinimumRange = minimumRange/3;//cm
const int calibratedMovementTime = 3500;//milli seconds
int robotWidth = 20;//cm
const int robotLength = 20;//cm
const int talkFrequency = 2000;//frequency in Hz
const int morseUnit = 200; //unit of morse
const int robotJamCheckTime = 1*60000; //milli seconds
const int solarCheckTime = 1000; //milli seconds
const boolean rotateMode = true;
const float sleepVoltage = 6.5;//volts
const float wakeVoltage = 7.0;//volts. Must be greater than sleepVoltage.
const int sleepCheckupTime = 300;//sec
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.

//Dont touch below stuff
VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);
VoltageSensor solarVoltageSensor(solarVoltageSensePin, smallR, bigR);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
MorseCode morseCode(speakerPin, talkFrequency, morseUnit);
unsigned long lastDirectionChangedTime = 0;
unsigned long lastSolarVoltageCheckTime = 0;
float lastSolarVoltage = 0;
boolean isMarkedForSleep = false;
unsigned long sleepCounter = 0;
boolean isBatteryChargedWhileSleeping_Cached = false;
boolean isIntruderDetected = false;

void setup() {
  Serial.begin (9600);
  pinMode(pirInterruptPin, INPUT);// define interrupt pin D2 as input to read interrupt received by PIR sensor

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
  if (!isBatteryDead()) {
    doBIOSManoeuvre();
  }

}

void loop() {

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

    //check if there is an emergency
    if (isEmergencyObstaclePresent()) {
      doEmergencyObstacleManoeuvre();
      return;
    }

    //check if there is an obstacle
    if (isObstaclePresent()) {
      doObstacleManoeuvre();
      return;
    }

    //check if the battery is running low
    if (false && isBatteryDying()) {
      doHarvestManoeuvre();
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
  lastDirectionChangedTime = 0;
  lastSolarVoltageCheckTime = 0;
  lastSolarVoltage = 0;
}

void markForWakeup() {
  morseCode.play("Awake");
  isMarkedForSleep = false;
  lastDirectionChangedTime = millis();
  lastSolarVoltageCheckTime = millis();
  lastSolarVoltage = solarVoltageSensor.senseVoltage();
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

boolean isBatteryDying() {
  return batteryVoltageSensor.senseVoltage() < wakeVoltage;
}

void intruderDetected() {
  isIntruderDetected = true;
}

boolean isObstaclePresent() {
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
  return (centerReading > 0 && centerReading <= minimumRange);
}

boolean isEmergencyObstaclePresent() {
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
  return (centerReading > 0 && centerReading <= emergencyMinimumRange);
}

void doObstacleManoeuvre() {
  if (decideOnRight()) {
    rotateRightByAngle(10 * random(0, 9));
  } else {
    rotateLeftByAngle(10 * random(0, 9));
  }
}

void doEmergencyObstacleManoeuvre() {
  tone(speakerPin, talkFrequency, 100);
  base.moveBackward((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  if (decideOnRight()) {
    rotateRightByAngle(10 * random(9, 18));
  } else {
    rotateLeftByAngle(10 * random(9, 18));
  }
}

void doJamManoeuvre() {
  base.stopAllMotion();
  morseCode.play("JAMMED");
  base.moveBackward((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  if (decideOnRight()) {
    rotateRightByAngle(10 * random(0, 36));
  } else {
    rotateLeftByAngle(10 * random(0, 36));
  }
}

void doIntruderManoeuvre() {
  morseCode.play("INTRUDER");
  isIntruderDetected = false;
}

void doHarvestManoeuvre() {
  if (millis() - lastSolarVoltageCheckTime > solarCheckTime) {
    float currentSolarVoltage = solarVoltageSensor.senseVoltage();
    Serial.println("Battery Voltage: "+ String(batteryVoltageSensor.senseVoltage()) + "| Solar Voltage: " + String(currentSolarVoltage));
    if ((millis() - lastDirectionChangedTime > solarCheckTime) && lastSolarVoltage > currentSolarVoltage) {
      if (decideOnRight()) {
        rotateRightByAngle(10 * random(9, 18));
      } else {
        rotateLeftByAngle(10 * random(9, 18));
      }
    }
    lastSolarVoltageCheckTime = millis();
    lastSolarVoltage = solarVoltageSensor.senseVoltage();
  }
}

void doSleepForEightSeconds() {
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time

  //Attach the PIR to activate intruder detection
  isIntruderDetected = false;
  attachInterrupt(digitalPinToInterrupt(pirInterruptPin), intruderDetected, RISING);

  //Begin the actual sleep
  __asm__  __volatile__("sleep");//in line assembler to go to sleep

  //Detach the PIR since we dont need intruder detection anymore
  detachInterrupt(digitalPinToInterrupt(pirInterruptPin));

  sleepCounter++;
}

void doBIOSManoeuvre() {
  //TODO: Need to improve this approach
  //Tell the voltage of battery
  float floatVoltage = batteryVoltageSensor.senseVoltage();
  morseCode.play(String(floatVoltage));

  //left
  rotateLeftByAngle(360);
  morseCode.play("Left");

  //right
  rotateRightByAngle(360);
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

void rotateRightByAngle(int angle) {
  if (rotateMode) {
    base.rotateRight((calibratedMovementTime / 360)*angle);
  } else {
    base.turnRight((calibratedMovementTime / 360)*angle);
  }
  lastDirectionChangedTime = millis();
}

void rotateLeftByAngle(int angle) {
  if (rotateMode) {
    base.rotateLeft((calibratedMovementTime / 360)*angle);
  } else {
    base.turnLeft((calibratedMovementTime / 360)*angle);
  }
  lastDirectionChangedTime = millis();
}

ISR(WDT_vect) {
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}//watchdog interrupt
