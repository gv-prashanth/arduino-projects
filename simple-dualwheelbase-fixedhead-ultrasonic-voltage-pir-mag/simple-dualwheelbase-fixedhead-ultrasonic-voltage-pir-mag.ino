#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DualWheelBase.h>
#include <MorseCode.h>
#include <DeepSleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//You can download the driver from https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_HMC5883_U.h> //SDA is A4, SCL is A5
//You can download the driver from https://github.com/adafruit/Adafruit_HMC5883_Unified
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
const int calibratedMovementTime = 7500;//milli seconds
const int talkFrequency = 2000;//frequency in Hz
const int morseUnit = 200; //unit of morse
const unsigned long robotJamCheckTime = 180000; //milli seconds
const float sleepVoltage = 3.0;//volts
const float wakeVoltage = 3.6;//volts. Must be greater than sleepVoltage.
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.
const float basePowerMultiplier = 0.5;//0.0 to 1.0
double Kp = 10, Ki = 2, Kd = 1; //Specify the links and initial tuning parameters

//Dont touch below stuff
unsigned long lastDirectionChangedTime = 0;
boolean isMarkedForSleep = false;
boolean isIntruderDetected = false;
float destinationHeading = 0.0;
double Setpoint, Input, Output;//Define Variables we'll be connecting to
VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DualWheelBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
MorseCode morseCode(speakerPin, talkFrequency, morseUnit);
DeepSleep deepSleep;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin (9600);
  pinMode(pirInterruptPin, INPUT);// define interrupt pin D2 as input to read interrupt received by PIR sensor
  pinMode(smartPowerPin, OUTPUT);

  Wire.begin();
  setupHMC5883L(); //setup the HMC5883L

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  //TODO: Setting base power to a fixed value. Need to make dynamic
  base.setPowerMultiplier(basePowerMultiplier);

  //Wake the robot
  markForWakeup();

  //Print the voltage of battery
  Serial.println("Battery Voltage: " + String(batteryVoltageSensor.senseVoltage()));

  //check if battery is low and skip bios dance
  if (!isBatteryDead()) {
    doBIOSManoeuvre();
  }

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
      destinationHeading = getHeading();
      return;
    }

    //incase of emergency stop
    if (isEmergencyObstaclePresent()) {
      doEmergencyObstacleManoeuvre();
      destinationHeading = getHeading();
      return;
    }

    //rotate left or right
    if (isAvoidableObstaclePresent()) {
      doAvoidableObstacleManoeuvre();
      destinationHeading = getHeading();
      return;
    }

    //go forward
    else {
      rotateOrSteerAndGoTowardsDestination();
      return;
    }

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

boolean isRightDecided() {
  return random(0, 2) < 1;
}

void doEmergencyObstacleManoeuvre() {
  base.stop();
  tone(speakerPin, talkFrequency, 100);
  base.goBackward();
  delay(calibratedMovementTime / M_PI);
  if (isRightDecided())
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
  if (isRightDecided())
    base.rotateRight();
  else
    base.rotateLeft();
  delay(random(0, 36) * 10 * (calibratedMovementTime / 360));
  base.stop();
  lastDirectionChangedTime = millis();
}

void doAvoidableObstacleManoeuvre() {
  if (isRightDecided())
    base.goForward(-255);
  else
    base.goForward(255);
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

float getHeading() {
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  "); Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  // float declinationAngle = 0.22;
  // heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;

}

void setupHMC5883L() {
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
}

void rotateOrSteerAndGoTowardsDestination() {
  float angleDiff = calculateAngularDifferenceVector();
  Setpoint = 0;
  Input = angleDiff;
  myPID.Compute();
  float powerDiff = Output;
  //Serial.println("PID Input: " + String(angleDiff) + " & Output: " + powerDiff + " will steer to fix the problem");
  //If powerDiff is positive i need to steer left
  //If powerDiff is negative i need to steer right
  if (powerDiff < -254) {
    base.rotateRight();
  } else if (powerDiff > 254) {
    base.rotateLeft();
  } else {
    base.goForward(powerDiff);
  }
}

float calculateAngularDifferenceVector() {
  //The angular error is calculated by actual - required. Further the easisest / closest direction is choosen as part of returning the value.
  float currentHeading = getHeading();
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
