#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DualWheelBase.h>
#include <MorseCode.h>
#include <DeepSleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//You can download the driver from https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_HMC5883_U.h>
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
const int emergencyObstacleRangeDetect = avoidableObstacleRange / 3; //cm
const int emergencyObstacleRangeAvoid = avoidableObstacleRange / 2; //cm
const int talkFrequency = 2000;//frequency in Hz
const int morseUnit = 200; //unit of morse
const unsigned long robotJamCheckTime = 120000; //milli seconds
const float sleepVoltage = 3.0;//volts
const float wakeVoltage = 3.6;//volts. Must be greater than sleepVoltage.
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.
const float basePower = 0.75;//0.0 to 1.0
double Kp = 60, Ki = 80, Kd = 20; //Specify the links and initial tuning parameters

//Dont touch below stuff
unsigned long *lastComandedDirectionChangeTime;
boolean isMarkedForSleep = false;
boolean isIntruderDetected = false;
float destinationHeading = 0.0;
double Setpoint, Input, Output;//Define Variables we'll be connecting to
int decidedDirection = 0;
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
    else if (isJamDetected()) {
      setJamDestination();
    }

    //incase of emergency stop
    else if (isEmergencyObstaclePresent()) {
      setEmergencyObstacleDestination();
    }

    //rotate left or right
    else if (isAvoidableObstaclePresent()) {
      setAvoidableObstacleDestination();
    }

    decidedDirection = 0;
    rotateOrSteerOrGoTowardsDestination();

  }

}

void markForSleep() {
  morseCode.play("SOS");
  isMarkedForSleep = true;
  digitalWrite(smartPowerPin, LOW);
}

//TODO: Need to get rid of marks logic
void markForWakeup() {
  morseCode.play("Awake");
  isMarkedForSleep = false;
  digitalWrite(smartPowerPin, HIGH);
  unsigned long temp = millis();
  lastComandedDirectionChangeTime = &temp;
}

boolean isBatteryCharged() {
  return batteryVoltageSensor.senseVoltage() > wakeVoltage;;
}

boolean isJamDetected() {
  return abs(millis() - *lastComandedDirectionChangeTime) > robotJamCheckTime;
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
  return (centerReading > 0 && centerReading <= emergencyObstacleRangeDetect);
}

boolean isEmergencyObstacleAvoidableDistanceYetToReach() {
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
  return (centerReading > 0 && centerReading <= emergencyObstacleRangeAvoid);
}

void setEmergencyObstacleDestination() {
  while (isEmergencyObstacleAvoidableDistanceYetToReach()) {
    base.goBackward();
  }
  if (decidedDirection == 0) {
    if (decideOnRight())
      decidedDirection = 1;
    else
      decidedDirection = -1;
  }
  if (decidedDirection = -1) {
    setLeftDestinationByAngle(random(9, 18) * 10);
  } else {
    setRightDestinationByAngle(random(9, 18) * 10);
  }
}

void setJamDestination() {
  base.goBackward();
  morseCode.play("J");
  if (decidedDirection == 0) {
    if (decideOnRight())
      decidedDirection = 1;
    else
      decidedDirection = -1;
  }
  if (decidedDirection = -1) {
    setLeftDestinationByAngle(random(0, 36) * 10);
  } else {
    setRightDestinationByAngle(random(0, 36) * 10);
  }
}

void setAvoidableObstacleDestination() {
  if (decidedDirection == 0) {
    if (decideOnRight())
      decidedDirection = 1;
    else
      decidedDirection = -1;
  }
  if (decidedDirection = -1) {
    setLeftDestinationByAngle(random(0, 9) * 10);
  } else {
    setRightDestinationByAngle(random(0, 9) * 10);
  }
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

//TODO: need to make bios dynamic and not rely on morseCodePlay
void doBIOSManoeuvre() {
  //left
  base.rotateLeft();
  morseCode.play("L");

  //right
  base.rotateRight();
  morseCode.play("R");

  //forward
  base.goForward();
  morseCode.play("F");

  //backward
  base.goBackward();
  morseCode.play("B");
}

boolean decideOnRight() {
  int randNumber = random(0, 2);
  if (randNumber < 1) {
    return true;
  } else {
    return false;
  }
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
  float declinationAngle = 0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
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

void rotateOrSteerOrGoTowardsDestination() {
  float angleDiff = calculateAngularDifferenceVector();
  Setpoint = 0;
  Input = angleDiff;
  myPID.Compute();
  float powerDiff = Output;
  Serial.println("PID Input: " + String(angleDiff) + " & Output: " + powerDiff + " will steer to fix the problem");
  //If powerDiff is negative i need to steer left
  //If powerDiff is positive i need to steer right
  if (powerDiff < -254) {
    base.rotateLeft();
  } else if (powerDiff > 254) {
    base.rotateRight();
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

void setLeftDestinationByAngle(int angle) {
  if (destinationHeading >= angle) {
    destinationHeading = destinationHeading - angle;
  } else {
    destinationHeading = 360 - (angle - destinationHeading);
  }
  unsigned long temp = millis();
  lastComandedDirectionChangeTime = &temp;
}

void setRightDestinationByAngle(int angle) {
  if (destinationHeading + angle < 360) {
    destinationHeading = destinationHeading + angle;
  } else {
    destinationHeading = (destinationHeading + angle) - 360;
  }
  unsigned long temp = millis();
  lastComandedDirectionChangeTime = &temp;
}