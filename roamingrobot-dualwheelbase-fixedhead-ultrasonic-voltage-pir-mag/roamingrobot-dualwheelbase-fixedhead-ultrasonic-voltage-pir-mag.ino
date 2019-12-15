#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DualWheelBase.h>
#include <MorseCode.h>
#include <DeepSleep.h>
#include <Wire.h>
#include <HMC5883L.h>
//You can download the driver from https://github.com/Seeed-Studio/Grove_3Axis_Digital_Compass_HMC5883L
#include <PID_v1.h>
//You can download the driver from https://github.com/br3ttb/Arduino-PID-Library/

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
//TODO: Need to delete the below variable
const int toDelete = 5000;//milli seconds
const int talkFrequency = 2000;//frequency in Hz
const int morseUnit = 200; //unit of morse
const unsigned long robotJamCheckTime = 180000; //milli seconds
const float sleepVoltage = 3.0;//volts
const float wakeVoltage = 3.6;//volts. Must be greater than sleepVoltage.
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.
const float basePower = 0.5;//0.0 to 1.0
double Kp = 2, Ki = 5, Kd = 1; //Specify the links and initial tuning parameters

//Dont touch below stuff
unsigned long lastDirectionChangedTime = 0;
boolean isMarkedForSleep = false;
boolean isIntruderDetected = false;
float destinationHeading;
double LSetpoint, LInput, LOutput;//Define Variables we'll be connecting to
double RSetpoint, RInput, ROutput;//Define Variables we'll be connecting to
VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DualWheelBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
MorseCode morseCode(speakerPin, talkFrequency, morseUnit);
DeepSleep deepSleep;
HMC5883L compass;
PID LmyPID(&LInput, &LOutput, &LSetpoint, Kp, Ki, Kd, DIRECT);
PID RmyPID(&RInput, &ROutput, &RSetpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin (9600);
  pinMode(pirInterruptPin, INPUT);// define interrupt pin D2 as input to read interrupt received by PIR sensor
  pinMode(smartPowerPin, OUTPUT);

  Wire.begin();
  compass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L

  LmyPID.SetMode(AUTOMATIC);
  RmyPID.SetMode(AUTOMATIC);

  //TODO: Setting base power to a fixed value. Need to make dynamic
  base.setPower(basePower);

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

  destinationHeading = getAvgHeading();
}

void loop() {

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

    //navigate the terrain
    else {
      setDestination();
      goTowardsDestination();
      return;
    }

  }

}

void setDestination() {
  //incase of emergency stop
  if (isEmergencyObstaclePresent()) {
    setEmergencyObstacleDestination();
  } else if (isAvoidableObstaclePresent()) {
    setAvoidableObstacleDestination();
  } else {
    //No need to set any directio. Just go towards last set direction.
  }
}

void markForSleep() {
  base.stop();
  morseCode.play("SOS");
  isMarkedForSleep = true;
  digitalWrite(smartPowerPin, LOW);
  //TODO: Need to get rid of below
  lastDirectionChangedTime = 0;
}

void markForWakeup() {
  morseCode.play("Awake");
  isMarkedForSleep = false;
  digitalWrite(smartPowerPin, HIGH);
  //TODO: Need to get rid of below
  lastDirectionChangedTime = millis();
}

boolean isBatteryCharged() {
  return batteryVoltageSensor.senseVoltage() > wakeVoltage;;
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

void setEmergencyObstacleDestination() {
  base.stop();
  tone(speakerPin, talkFrequency, 100);
  base.goBackward();
  delay(toDelete);
  if (decideOnRight())
    setRightDestinationByAngle(random(9, 18) * 10);
  else
    setLeftDestinationByAngle(random(9, 18) * 10);
  lastDirectionChangedTime = millis();
}

void setAvoidableObstacleDestination() {
  if (decideOnRight())
    setRightDestinationByAngle(random(0, 9) * 10);
  else
    setLeftDestinationByAngle(random(0, 9) * 10);
  lastDirectionChangedTime = millis();
}

void setLeftDestinationByAngle(int angle) {
  if (destinationHeading >= angle) {
    destinationHeading = destinationHeading - angle;
  } else {
    destinationHeading = 360 - (angle - destinationHeading);
  }
}

void setRightDestinationByAngle(int angle) {
  if (destinationHeading + angle < 360) {
    destinationHeading = destinationHeading + angle;
  } else {
    destinationHeading = (destinationHeading + angle) - 360;
  }
}

void doJamManoeuvre() {
  base.stop();
  morseCode.play("JAMMED");
  base.goBackward();
  delay(toDelete);
  if (decideOnRight())
    setRightDestinationByAngle(random(0, 36) * 10);
  else
    setLeftDestinationByAngle(random(0, 36) * 10);
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
}

void doBIOSManoeuvre() {
  //left
  base.rotateLeft();
  delay(toDelete);
  base.stop();
  morseCode.play("Left");

  //right
  base.rotateRight();
  delay(toDelete);
  base.stop();
  morseCode.play("Right");

  //forward
  base.goForward();
  delay(toDelete);
  base.stop();
  morseCode.play("Forward");

  //backward
  base.goBackward();
  delay(toDelete);
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

float getAvgHeading() {
  int NumOfAttempts = 10;
  float toReturnAvg = 0.0;
  for (int i = 1; i <= NumOfAttempts; i++) {
    float thisHeadingToAvg = getHeading();
    toReturnAvg = toReturnAvg + thisHeadingToAvg;
    delay(10);
  }
  return toReturnAvg / NumOfAttempts;
}

float getHeading() {
  //Get the reading from the HMC5883L and calculate the heading
  MagnetometerScaled scaled = compass.readScaledAxis(); //scaled values from compass.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  Serial.println("Mag Reading: " + String(heading * RAD_TO_DEG));
  return heading * RAD_TO_DEG; //radians to degrees
}

void setupHMC5883L() {
  //Setup the HMC5883L, and check for errors
  int error;
  error = compass.setScale(1.3); //Set the scale of the compass.
  if (error != 0) Serial.println(compass.getErrorText(error)); //check if there is an error, and print if so

  error = compass.setMeasurementMode(MEASUREMENT_CONTINUOUS); // Set the measurement mode to Continuous
  if (error != 0) Serial.println(compass.getErrorText(error)); //check if there is an error, and print if so
}

void goTowardsDestination() {
  float rightAngleDiff = rightAngularDifference();
  float leftAngleDiff = leftAngularDifference();
  if (rightAngleDiff < leftAngleDiff) {
    //easier to fix by steering left
    LSetpoint = 0;
    LInput = getAngularDiffIn255Scale(rightAngleDiff);
    LmyPID.Compute();
    Serial.println("Left Input: " + String(getAngularDiffIn255Scale(rightAngleDiff)) + " & Output: " + LOutput);
    base.steerLeft(LOutput);
  } else {
    //easier to fix by steering right
    RSetpoint = 0;
    RInput = getAngularDiffIn255Scale(leftAngleDiff);
    RmyPID.Compute();
    Serial.println("Right Input: " + String(getAngularDiffIn255Scale(leftAngleDiff)) + " & Output: " + ROutput);
    base.steerRight(ROutput);
  }
}

float rightAngularDifference() {
  float toReturn = -1.0;
  float currentHeading = getAvgHeading();
  if (destinationHeading > 180 && destinationHeading < 360 && currentHeading > 0 && currentHeading < 180) {
    toReturn = (360.0 - destinationHeading) + currentHeading;
  } else if (currentHeading > destinationHeading) {
    toReturn = currentHeading - destinationHeading;
  }
  return toReturn;
}

float leftAngularDifference() {
  float toReturn = -1.0;
  float currentHeading = getAvgHeading();
  if (destinationHeading > 0 && destinationHeading < 180 && currentHeading > 180 && currentHeading < 360) {
    toReturn = (360.0 - currentHeading) + destinationHeading;
  } else if (currentHeading < destinationHeading) {
    toReturn = destinationHeading - currentHeading;
  }
  return toReturn;
}

//TODO: Need to get rid if this in future using PID
float getAngularDiffIn255Scale(float val) {
  return map(val, 0, 360, 0, 255);
}
