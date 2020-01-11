#include <DualWheelBase.h>
#include <Wire.h>
#include <PID_v1.h> //You can download the driver from https://github.com/br3ttb/Arduino-PID-Library/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //You can download the driver from https://github.com/electroniccats/mpu6050
#include "Wire.h"

//Pin Configuration
const int leftWheelForwardPin = 6;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 10;//10
const int rightWheelBackwardPin = 11;//11
const int smartPowerPin = 12; //can be used to sleep and wake peripherals

//functional Configuration
const float basePower = 0.8;//0.0 to 1.0
double Kp = 2, Ki = 0.2, Kd = 0.6; //Specify the links and initial tuning parameters

//Dont touch below stuff
float destinationHeading;
double Setpoint, Input, Output;//Define Variables we'll be connecting to

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

DualWheelBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
MPU6050 mpu;

void setup() {
  Serial.begin (9600);

  pinMode(smartPowerPin, OUTPUT);
  digitalWrite(smartPowerPin, HIGH);
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  //TODO: Setting base power to a fixed value. Need to make dynamic
  base.setPowerMultiplier(basePower);

  setupMPU();
  stabilizeMPU();
  
  destinationHeading = getHeading();
  Serial.println("BIOS complete. Setting heading to " + String(destinationHeading));
}

void loop() {
  populateYPR();
  goTowardsDestination();
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
    while (true);
  }
}

void stabilizeMPU() {
  Serial.println("Stabilizing MPU...");
  unsigned long currentTime = millis();
  while (millis() - currentTime < 30000) {
    populateYPR();
  }
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

void goTowardsDestination() {
  float angleDiff = calculateAngularDifferenceVector();
  Setpoint = 0;
  Input = angleDiff;
  myPID.Compute();
  //If powerDiff is positive i need to steer left
  //If powerDiff is negative i need to steer right
  float powerDiff = Output;
  //Serial.println("PID Input: " + String(angleDiff) + " & Output: " + powerDiff + " will steer to fix the problem");
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
