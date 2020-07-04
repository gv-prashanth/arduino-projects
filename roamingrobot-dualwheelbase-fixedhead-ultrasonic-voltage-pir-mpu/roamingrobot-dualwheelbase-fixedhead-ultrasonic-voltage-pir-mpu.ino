#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DualWheelBase.h>
#include <MorseCode.h>
#include <PID_v1.h> //You can download the driver from https://github.com/br3ttb/Arduino-PID-Library/

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 11;//10
const int rightWheelBackwardPin = 10;//11
const int smartPowerPin = 8; //can be used to sleep and wake peripherals
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 6;
const int batteryVoltageSensePin = A2;//A2 incase you want to detect from dedicated pin. -1 incase you want to detect from vcc.
const int pirInterruptPin = 3;//pin 3 only should be used

//functional Configuration
const int avoidableObstacleRange = 60;//cm
const int emergencyObstacleRange = avoidableObstacleRange / 3; //cm
const float emergencyPitchRollRange = 5;//degrees
const int timeToStickRightLeftDecission = 5000;//milli seconds
const int talkFrequency = 2700;//frequency in Hz
const int morseUnit = 200; //unit of morse
const unsigned long robotJamCheckTime = 120000; //milli seconds
const int baseMovementTime = 1500;//milli seconds
const float sleepVoltage = 3.3;//volts
const float wakeVoltage = 3.7;//volts. Must be greater than sleepVoltage.
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.
double Kp = 8.0, Ki = 1.0, Kd = 2.0; //Specify the links and initial tuning parameters

//Dont touch below stuff
unsigned long lastComandedDirectionChangeTime, overrideForwardUntill, lastRightLeftDecidedTime;
boolean isMarkedForSleep, isIntruderDetected, isRightDecidedCached;
double Setpoint, Input, Output;//Define Variables we'll be connecting to
float emergencyPitchOffset, emergencyRollOffset, destinationHeading, currentHeading;
float ypr[3];// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int centerReading;

VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin, &centerReading);
DualWheelBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
MorseCode morseCode(speakerPin, talkFrequency, morseUnit);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  pinMode(pirInterruptPin, INPUT);// define interrupt pin as input to read interrupt received by PIR sensor
  pinMode(smartPowerPin, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  //Wake the robot
  markForWakeup();

  //Print the voltage of battery
  Serial.println("Battery Voltage: " + String(batteryVoltageSensor.senseVoltage()));

  //check if battery is low and skip bios dance
  if (!isBatteryDead()) {
    doBIOSManoeuvre();
  }

  setupMPU();
  stabilizeMPU();
}

void loop() {
  //NOTE: Never write anything at the start of loop. Since it will impact the sleep power consumption

  if (isMarkedForSleep) {

    //check if battery is charged and wake up
    if (isBatteryCharged()) {
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

    //TODO: Dont quite like it here
    populateYPR();
    populateHeading();
    ultrasonicSensor.populateReading();

    //check if battery is low and go to sleep
    if (isBatteryDead()) {
      markForSleep();
      return;
    }

    //TODO: Need to better check if there is a robot jam
    else if (isJamDetected()) {
      tone(speakerPin, talkFrequency, 1000);
      markForForwardOverride(baseMovementTime);
      setJamDestination();
    }

    //incase of excessive pitch or roll
    //below is true means im going forward
    else if (isEmergencyPitchRollSituation() && !isForwardOverridden() && isClimbingPitch()) {
      //so lets go backward
      tone(speakerPin, talkFrequency, 100);
      markForForwardOverride(baseMovementTime);
      setEmergencyDestination();
    }

    //incase of excessive pitch or roll
    //below is true means im going backward
    //TODO: Not really sure if this is working. Need to test properly
    else if (isEmergencyPitchRollSituation() && isForwardOverridden() && !isClimbingPitch()) {
      //so lets not go backward. You can stop going backward by removing mark
      tone(speakerPin, talkFrequency, 100);
      markForForwardOverride(0);
      setAvoidableObstacleDestination();
    }

    //incase of emergency
    else if (isObstacleWithinEmergencyDistance()) {
      tone(speakerPin, talkFrequency, 100);
      markForForwardOverride(baseMovementTime);
      setEmergencyDestination();
    }

    //Only do this when you are moving forward
    else if (isObstacleWithinAvoidableDistance()) {
      //below is true means im going forward
      if (!isForwardOverridden())
        setAvoidableObstacleDestination();
    }

    rotateOrSteerAndGoTowardsDestination();

  }

}

void markForSleep() {
  morseCode.play("SOS");
  isMarkedForSleep = true;
  digitalWrite(smartPowerPin, LOW);
}

//TODO: Need to get rid of marks logic
void markForWakeup() {
  morseCode.play("A");
  isMarkedForSleep = false;
  digitalWrite(smartPowerPin, HIGH);
  lastComandedDirectionChangeTime = millis();
  overrideForwardUntill = millis();
}

void markForForwardOverride(unsigned long overrideTime) {
  if (overrideForwardUntill < millis())
    overrideForwardUntill = millis() + overrideTime;
}

boolean isClimbingPitch() {
  return ypr[1] >= 0;
}

boolean isForwardOverridden() {
  return overrideForwardUntill > millis();
}

boolean isBatteryCharged() {
  return batteryVoltageSensor.senseVoltage() > wakeVoltage;
}

boolean isJamDetected() {
  return abs(millis() - lastComandedDirectionChangeTime) > robotJamCheckTime;
}

boolean isBatteryDead() {
  return batteryVoltageSensor.senseVoltage() < sleepVoltage;
}

void intruderDetected() {
  isIntruderDetected = true;
}

boolean isObstacleWithinAvoidableDistance() {
  return (centerReading > emergencyObstacleRange && centerReading <= avoidableObstacleRange);
}

boolean isEmergencyPitchRollSituation() {
  float currentPitchWithOffset = ypr[1] - emergencyPitchOffset;
  float currentRollWithOffset = ypr[2] - emergencyRollOffset;
  boolean safe = (-emergencyPitchRollRange < currentPitchWithOffset && currentPitchWithOffset < emergencyPitchRollRange &&
                  -emergencyPitchRollRange < currentRollWithOffset && currentRollWithOffset < emergencyPitchRollRange);
  return !safe;
}

boolean isObstacleWithinEmergencyDistance() {
  return (centerReading > 0 && centerReading <= emergencyObstacleRange);
}

boolean isRightDecided() {
  if (millis() - lastRightLeftDecidedTime > timeToStickRightLeftDecission) {
    isRightDecidedCached = (random(0, 2) < 1);
    lastRightLeftDecidedTime = millis();
  }
  return isRightDecidedCached;
}

void setEmergencyDestination() {
  if (isRightDecided())
    setRightDestinationByAngle(random(9, 18) * 10);
  else
    setLeftDestinationByAngle(random(9, 18) * 10);
}

void setJamDestination() {
  if (isRightDecided())
    setRightDestinationByAngle(random(1, 36) * 10);
  else
    setLeftDestinationByAngle(random(1, 36) * 10);
}

void setAvoidableObstacleDestination() {
  if (isRightDecided())
    setRightDestinationByAngle(random(1, 9) * 10);
  else
    setLeftDestinationByAngle(random(1, 9) * 10);
}

void doIntruderManoeuvre() {
  morseCode.play("INTRUDER");
  isIntruderDetected = false;
}

void doBIOSManoeuvre() {
  //left
  morseCode.play("L");
  base.rotateLeft();
  delay(baseMovementTime);
  base.stop();

  //right
  morseCode.play("R");
  base.rotateRight();
  delay(baseMovementTime);
  base.stop();

  //forward
  morseCode.play("F");
  base.goForward();
  delay(baseMovementTime);
  base.stop();

  //backward
  morseCode.play("B");
  base.goBackward();
  delay(baseMovementTime);
  base.stop();

}

float populateHeading() {
  if (ypr[0] > 0)
    return ypr[0];
  else
    return (180 - abs(ypr[0])) + 180;
}

void stabilizeMPU() {
  Serial.println("Stabilizing MPU...");
  unsigned long currentTime = millis();
  tone(speakerPin, talkFrequency, 1000);
  while (millis() - currentTime < 30000) {
    populateYPR();
  }
  tone(speakerPin, talkFrequency, 1000);
  emergencyPitchOffset = ypr[1];
  emergencyRollOffset = ypr[2];
  Serial.println("Using pitch offset as " + String(emergencyPitchOffset));
  Serial.println("Using roll offset as " + String(emergencyRollOffset));
  Serial.println("MPU stabilization complete");
}

void rotateOrSteerAndGoTowardsDestination() {
  float angleDiff = calculateAngularDifferenceVector();
  Setpoint = 0;
  Input = angleDiff;
  myPID.Compute();
  //If powerDiff is positive i need to steer left
  //If powerDiff is negative i need to steer right
  float powerDiff = Output;
  //Serial.println("PID Input: " + String(angleDiff) + " & Output: " + powerDiff + " will steer to fix the problem");
  if (isForwardOverridden()) {
    base.goBackward();
  } else if (powerDiff < -254) {
    base.rotateRight();
  } else if (powerDiff > 254) {
    base.rotateLeft();
  } else {
    base.goForward(powerDiff);
  }
}

float calculateAngularDifferenceVector() {
  //The angular error is calculated by actual - required. Further the easisest / closest direction is choosen as part of returning the value.
  if (currentHeading >= destinationHeading) {
    float left = currentHeading - destinationHeading;
    float right = (360.0 - currentHeading) + destinationHeading;
    if (left < right)
      return -1 * left;
    else
      return right;
  } else {
    float left = (360.0 - destinationHeading) + currentHeading;
    float right = destinationHeading - currentHeading;
    if (left < right)
      return -1 * left;
    else
      return right;
  }
}

void setLeftDestinationByAngle(int angle) {
  if (currentHeading >= angle) {
    destinationHeading = currentHeading - angle;
  } else {
    destinationHeading = 360 - (angle - currentHeading);
  }
  lastComandedDirectionChangeTime = millis();
}

void setRightDestinationByAngle(int angle) {
  if (currentHeading + angle < 360) {
    destinationHeading = currentHeading + angle;
  } else {
    destinationHeading = (currentHeading + angle) - 360;
  }
  lastComandedDirectionChangeTime = millis();
}
