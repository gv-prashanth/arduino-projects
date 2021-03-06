#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DualWheelBase.h>
#include <MorseCode.h>
#include <DeepSleep.h>
#include <PID_v1.h> //You can download the driver from https://github.com/br3ttb/Arduino-PID-Library/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //You can download the driver from https://github.com/electroniccats/mpu6050
#include "Wire.h"

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
unsigned long lastComandedDirectionChangeTime = 0;
unsigned long overrideForwardUntill = 0;
unsigned long lastRightLeftDecidedTime = 0;
boolean isMarkedForSleep = false;
boolean isIntruderDetected = false;
boolean isRightDecidedCached = false;
float destinationHeading = 0.0;
double Setpoint, Input, Output;//Define Variables we'll be connecting to
float emergencyPitchOffset = 0;
float emergencyRollOffset = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DualWheelBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
MorseCode morseCode(speakerPin, talkFrequency, morseUnit);
DeepSleep deepSleep;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
MPU6050 mpu;

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
  calculatePitchRollBodyOffset();

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
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
  return (centerReading > emergencyObstacleRange && centerReading <= avoidableObstacleRange);
}

boolean isEmergencyPitchRollSituation() {
  float currentPitchWithOffset = (ypr[1] * 180 / M_PI) - emergencyPitchOffset;
  float currentRollWithOffset = (ypr[2] * 180 / M_PI) - emergencyRollOffset;
  boolean safe = (-emergencyPitchRollRange < currentPitchWithOffset && currentPitchWithOffset < emergencyPitchRollRange &&
                  -emergencyPitchRollRange < currentRollWithOffset && currentRollWithOffset < emergencyPitchRollRange);
  return !safe;
}

boolean isObstacleWithinEmergencyDistance() {
  int centerReading = (int) ultrasonicSensor.obstacleDistance();
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

float getHeading() {
  if (ypr[0] > 0) {
    //Serial.println(ypr[0] * 180 / M_PI);
    return ypr[0] * 180 / M_PI;
  }
  else {
    //Serial.println((180 - abs(ypr[0] * 180 / M_PI)) + 180);
    return (180 - abs(ypr[0] * 180 / M_PI)) + 180;
  }
}

//setup the mpu
void setupMPU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    morseCode.play("DMP Failed");
    while (true);
  }
}

void stabilizeMPU() {
  Serial.println("Stabilizing MPU...");
  unsigned long currentTime = millis();
  tone(speakerPin, talkFrequency, 1000);
  while (millis() - currentTime < 30000) {
    populateYPR();
  }
  tone(speakerPin, talkFrequency, 1000);
  Serial.println("MPU stabilization complete");
}

//dmp interrupt
void dmpDataReady() {
  mpuInterrupt = true;
}

//populates the ypr based on interrupt condition
void populateYPR() {
  // wait for MPU interrupt or extra packet(s) available
  if (mpuInterrupt || fifoCount > packetSize) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//      Serial.print("ypr\t");
//      Serial.print(ypr[0] * 180 / M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[1] * 180 / M_PI);
//      Serial.print("\t");
//      Serial.println(ypr[2] * 180 / M_PI);
    }
  }
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
  float currentHeading = getHeading();
  if (currentHeading >= angle) {
    destinationHeading = currentHeading - angle;
  } else {
    destinationHeading = 360 - (angle - currentHeading);
  }
  lastComandedDirectionChangeTime = millis();
}

void setRightDestinationByAngle(int angle) {
  float currentHeading = getHeading();
  if (currentHeading + angle < 360) {
    destinationHeading = currentHeading + angle;
  } else {
    destinationHeading = (currentHeading + angle) - 360;
  }
  lastComandedDirectionChangeTime = millis();
}

void calculatePitchRollBodyOffset() {
  populateYPR();
  emergencyPitchOffset = ypr[1] * 180 / M_PI;
  emergencyRollOffset = ypr[2] * 180 / M_PI;
  Serial.println("Using pitch offset as " + String(emergencyPitchOffset));
  Serial.println("Using roll offset as " + String(emergencyRollOffset));
}
