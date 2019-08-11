#include <AnalogBase.h>
#include <Servo.h>
#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <Wire.h>
#include <HMC5883L.h>

//Pin Configuration
const int leftWheelForwardPin = 11;
const int leftWheelBackwardPin = 5;
const int rightWheelForwardPin = 3;
const int rightWheelBackwardPin = 6;
const int speakerPin = 2;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int servoPin = 9;
const int voltagePin = A2;
const int leftWheelEnablePin = 12;
const int rightWheelEnablePin = 4;

//functional Configuration
const int minDiffForDecissionChange = 50;//cm
const int minimumRange = 30;//cm
const int baseMovementTime = 400;//milli seconds
const int robotWidth = 20;//cm
const float permittedAngularVariance = 5.0;

//Dont touch below stuff
Servo microServo;
VoltageSensor voltageSensor(voltagePin);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
AnalogBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
HMC5883L compass;
float destinationHeading;
float directionChangeCounter;

void setup() {
  Serial.begin (9600);
  pinMode(leftWheelEnablePin, OUTPUT);
  pinMode(rightWheelEnablePin, OUTPUT);
  digitalWrite(leftWheelEnablePin, HIGH);
  digitalWrite(rightWheelEnablePin, HIGH);
  
  checkHeadPosition();
  checkBatteryVoltage();
  checkBaseHeadDirections();
  
  Wire.begin();
  compass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L
  directionChangeCounter = 0;
  destinationHeading = getAvgHeading();
}

void loop() {
  setDestination();
  goTowardsDestination();
}

float rightAngularDifference(float currentHeading,float destinationHeading){
  float toReturn = -1.0;
  if(destinationHeading>180 && destinationHeading<360 && currentHeading>0 && currentHeading<180){
     toReturn = (360.0-destinationHeading)+currentHeading;
  }else if(currentHeading > destinationHeading){
    toReturn = currentHeading - destinationHeading;
  }
  return toReturn;
}

float leftAngularDifference(float currentHeading,float destinationHeading){
  float toReturn = -1.0;
  if(destinationHeading>0 && destinationHeading<180 && currentHeading>180 && currentHeading<360){
     toReturn = (360.0-currentHeading)+destinationHeading;
  }else if(currentHeading < destinationHeading){
    toReturn = destinationHeading - currentHeading;
  }
  return toReturn;
}

void checkHeadPosition(){
  scanAndGetReading(90);
}

void checkBatteryVoltage(){
  int intVoltage = voltageSensor.senseVoltage();
  for(int i=0;i<intVoltage; i++){
    tone(speakerPin, 5000, 200);
    delay(400);
  }
}

void checkBaseHeadDirections(){
  base.rotateLeft(1.0);
  delay(baseMovementTime);
  base.stopAllMotion();
  delay(400);
  base.rotateRight(1.0);
  delay(baseMovementTime);
  base.stopAllMotion();
  delay(400);
  scanAndGetReading(140);
  delay(400);
  scanAndGetReading(90);
  delay(400);
  base.rotateRight(1.0);
  delay(baseMovementTime);
  base.stopAllMotion();
  delay(400);
  base.rotateLeft(1.0);
  delay(baseMovementTime);
  base.stopAllMotion();
  delay(400);
  scanAndGetReading(40);
  delay(400);  
  scanAndGetReading(90);
  delay(400);
  base.goForward(1.0,1.0);
  delay(baseMovementTime);
  base.stopAllMotion();
  delay(400);
  base.goBackward(1.0,1.0);
  delay(baseMovementTime);
  base.stopAllMotion();
  delay(400);
}

void obstacleTooCloseEmergencyStop(){
  tone(speakerPin, 2000, 100);
  base.goBackward(1.0,1.0);
  delay(baseMovementTime);
  base.stopAllMotion();
  if(decideOnRight()){
    base.rotateRight(1.0);
    delay(baseMovementTime);
    base.stopAllMotion();
  }else{
    base.rotateLeft(1.0);
    delay(baseMovementTime);
    base.stopAllMotion();
  }
  destinationHeading = getAvgHeading();
}

int scanAndGetReading(int angle){
  microServo.attach(servoPin);
  delay(5);
  microServo.write(angle);
  delay(200);
  microServo.detach();
  delay(5);
  int reading = (int) ultrasonicSensor.obstacleDistance();
  return reading;
}

boolean decideOnRight(){
  int randNumber = random(0, 2);
  if(randNumber <1){
    return true;
  }else{
    return false;
  }
}

float calculatePowerMultiplier(float lowerRange, float higherRange){
  float voltage = voltageSensor.senseVoltage();
  float powerMultiplier; 
  if(voltage >= lowerRange){
    powerMultiplier = 1 - 0.4*((voltage-lowerRange)/(higherRange-lowerRange));
    //when higherRange then 60% and when lowerRange then 100%
  }else{
    powerMultiplier = 1;
  }
  //Serial.print(powerMultiplier);
  //Serial.println (" is what im using currently");
  return powerMultiplier;
}

void setupHMC5883L(){
 //Setup the HMC5883L, and check for errors
 int error; 
 error = compass.SetScale(1.3); //Set the scale of the compass.
 if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so

 error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
 if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
}

float getAvgHeading(){
  int NumOfAttempts = 10;
  float toReturnAvg = 0.0;
  for(int i=1;i<=NumOfAttempts;i++){
    float thisHeadingToAvg = getHeading();
    toReturnAvg = toReturnAvg + thisHeadingToAvg;
    delay(10);
  }
  return toReturnAvg/NumOfAttempts;
}

float getHeading(){
 //Get the reading from the HMC5883L and calculate the heading
 MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.
 float heading = atan2(scaled.YAxis, scaled.XAxis);

 // Correct for when signs are reversed.
 if(heading < 0) heading += 2*PI;
 if(heading > 2*PI) heading -= 2*PI;

 return heading * RAD_TO_DEG; //radians to degrees
}

float getDirectionCorrectionDiff(float val){
  if(val <= 30){
    return map(val, 1, 30, 100, 50)/100.0;
  }
  return 0.5;
}

void goTowardsDestination(){
  float currentHeading = getAvgHeading();
  float rightDiff = rightAngularDifference(currentHeading,destinationHeading);
  float leftDiff = leftAngularDifference(currentHeading,destinationHeading);
  float powerMultiplier = calculatePowerMultiplier(9.0,13.0);
  if(rightDiff>permittedAngularVariance){
    //steering towards right.. fix it
    float calculatedPowerDiff = getDirectionCorrectionDiff(rightDiff);
    Serial.print(rightDiff);Serial.print(" variance.");Serial.print(powerMultiplier*calculatedPowerDiff);Serial.print(" left.");Serial.print(powerMultiplier);Serial.print(" right.");Serial.println();
    base.goForward(powerMultiplier*calculatedPowerDiff,powerMultiplier);
  }else if(leftDiff>permittedAngularVariance){
    //steering towards left.. fix it
    float calculatedPowerDiff = getDirectionCorrectionDiff(leftDiff);
    Serial.print(leftDiff);Serial.print(" variance.");Serial.print(powerMultiplier*calculatedPowerDiff);Serial.print(" right.");Serial.print(powerMultiplier);Serial.print(" left.");Serial.println();
    base.goForward(powerMultiplier,powerMultiplier*calculatedPowerDiff);
  }else{
    base.goForward(powerMultiplier,powerMultiplier);
  }
}

void setDestination(){
  int centerReading = (int) scanAndGetReading(90);
  if(centerReading>0 && centerReading<=minimumRange){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int leftReading = scanAndGetReading(140);
  if(leftReading>0 && leftReading<=robotWidth){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int rightReading = scanAndGetReading(40);
  if(rightReading>0 && rightReading<=robotWidth){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int centerReading2 = scanAndGetReading(90);
  if(centerReading2>0 && centerReading2<=minimumRange){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int leftDecissionValue = leftReading-centerReading;
  int rightDecissionValue = rightReading-centerReading2;
  if(directionChangeCounter > 1){
    if((leftDecissionValue>minDiffForDecissionChange)&&(leftDecissionValue>=rightDecissionValue)){
      //take left turn
      if(destinationHeading >= 30){
        destinationHeading = destinationHeading-30;
      }else{
        destinationHeading = 360 - (30-destinationHeading);
      }
      tone(speakerPin, 5000, 100);
    }else if((rightDecissionValue>minDiffForDecissionChange)&&(rightDecissionValue>leftDecissionValue)){
      //take right turn
      if(destinationHeading+30<360){
        destinationHeading = destinationHeading+30;
      }else{
        destinationHeading = (destinationHeading+30) - 360;
      }
      tone(speakerPin, 5000, 100);
    }
  directionChangeCounter = 0;
  }
  directionChangeCounter++;
  //directionChangeCounter is implemented to prevent continuous switching of destination
}
/*
tuning required for voltage stabilization - DONE
forward perfect using magnetometer - DONE ALMOST
some issue with power... need to analyze, arduino resets - problem due to same powersupply, when in low, arduino resets
SDA is A4
SCL is A5
*/
