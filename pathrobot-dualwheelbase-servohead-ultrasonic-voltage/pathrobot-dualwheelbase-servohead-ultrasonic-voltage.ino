#include <DualWheelBase.h>
#include <Servo.h>
#include <VoltageSensor.h>
#include <UltrasonicSensor.h>

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
const int leftWheelEnablePin = 12;//Ignore if your motor driver does not have an enable pin
const int rightWheelEnablePin = 4;//Ignore if your motor driver does not have an enable pin

//functional Configuration
const int minDiffForDecissionChange = 50;//cm
const int minimumRange = 30;//cm
const int baseMovementTime = 500;//milli seconds
const int robotWidth = 20;//cm
const int headTurnAngle = 40;//angle in degrees
const int talkFrequency = 2000;//frequency in Hz
const int shoutFrequency = 5000;//frquency in Hz
const float highestWorkingVoltageSpecificationOfBatteryPack = 13.0; //Volts
const float lowestWorkingVoltageSpecificationOfBatteryPack = 9.0; //Volts
const float powerSuppresorFactorAtHighestVoltage = 0.4;//if you pass 0.4 then at highestWorkingVoltageSpecificationOfBatteryPack 60% power and lowestWorkingVoltageSpecificationOfBatteryPack 100% power
const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 10000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.

//Dont touch below stuff
Servo microServo;
VoltageSensor voltageSensor(voltagePin, smallR, bigR);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DualWheelBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);

void setup() {
  Serial.begin (9600);
  pinMode(leftWheelEnablePin, OUTPUT);
  pinMode(rightWheelEnablePin, OUTPUT);
  digitalWrite(leftWheelEnablePin, HIGH);
  digitalWrite(rightWheelEnablePin, HIGH);
  
  checkHeadPosition();
  checkBatteryVoltage();
  checkBaseHeadDirections();
}

void loop() {
  float powerMultiplier = calculatePowerMultiplier(lowestWorkingVoltageSpecificationOfBatteryPack, highestWorkingVoltageSpecificationOfBatteryPack, powerSuppresorFactorAtHighestVoltage);
  base.setPowerMultiplier(powerMultiplier);
  
  int centerReading = (int) scanAndGetReading(90);
  if(centerReading>0 && centerReading<=minimumRange){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int leftReading = scanAndGetReading(180-headTurnAngle);
  if(leftReading>0 && leftReading<=robotWidth){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int rightReading = scanAndGetReading(headTurnAngle);
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
  
  if((leftDecissionValue>minDiffForDecissionChange)&&(leftDecissionValue>=rightDecissionValue)){
    //take left turn
    tone(speakerPin, talkFrequency, 200);
    base.rotateLeft();
    delay(baseMovementTime);
  }else if((rightDecissionValue>minDiffForDecissionChange)&&(rightDecissionValue>leftDecissionValue)){
    //take right turn
    tone(speakerPin, talkFrequency, 200);
    base.rotateRight();
    delay(baseMovementTime);
  }
  
  //go forward
  base.goForward();
}

void checkHeadPosition(){
  scanAndGetReading(90);
}

void checkBatteryVoltage(){
  int intVoltage = voltageSensor.senseVoltage();
  for(int i=0;i<intVoltage; i++){
    tone(speakerPin, talkFrequency, 200);
    delay(400);
  }
}

void checkBaseHeadDirections(){
  base.rotateLeft();
  delay(baseMovementTime);
  base.stop();
  delay(400);
  base.rotateRight();
  delay(baseMovementTime);
  base.stop();
  delay(400);
  scanAndGetReading(180-headTurnAngle);
  delay(400);
  scanAndGetReading(90);
  delay(400);
  base.rotateRight();
  delay(baseMovementTime);
  base.stop();
  delay(400);
  base.rotateLeft();
  delay(baseMovementTime);
  base.stop();
  delay(400);
  scanAndGetReading(headTurnAngle);
  delay(400);  
  scanAndGetReading(90);
  delay(400);
  base.goForward();
  delay(baseMovementTime);
  base.stop();
  delay(400);
  base.goBackward();
  delay(baseMovementTime);
  base.stop();
  delay(400);
}

void obstacleTooCloseEmergencyStop(){
  tone(speakerPin, shoutFrequency, 100);
  base.goBackward();
  delay(baseMovementTime);
  base.stop();
  if(decideOnRight()){
    base.rotateRight();
    delay(baseMovementTime);
    base.stop();
  }else{
    base.rotateLeft();
    delay(baseMovementTime);
    base.stop();
  }
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

float calculatePowerMultiplier(float lowerRange, float higherRange, float powerSuppresorFactorAtHighestVoltage){
  float voltage = voltageSensor.senseVoltage();
  float powerMultiplier; 
  if(voltage >= lowerRange){
    powerMultiplier = 1 - powerSuppresorFactorAtHighestVoltage*((voltage-lowerRange)/(higherRange-lowerRange));
    //if you pass 0.4 then at higherRange 60% power and lowerRange 100% power
  }else{
    powerMultiplier = 1;
  }
  //Serial.print(powerMultiplier);
  //Serial.println (" is what im using currently");
  return powerMultiplier;
}
