#include <Servo.h>
#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DigitalBase.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 10;//10
const int rightWheelBackwardPin = 11;//11
const int speakerPin = 6;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int servoPin = 3;
const int voltagePin = A2;

//functional Configuration
const int minDiffForDecissionChange = 50;//cm
const int minimumRange = 25;//cm
const int baseMovementTime = 1000;//milli seconds
const int robotWidth = 12;//cm
const int headTurnAngle = 20;//angle in degrees
const int talkFrequency = 2000;//frequency in Hz
const int shoutFrequency = 5000;//frquency in Hz

//Dont touch below stuff
Servo microServo;
VoltageSensor voltageSensor(voltagePin);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);

void setup() {
  Serial.begin (9600);
  checkHeadPosition();
  checkBatteryVoltage();
  checkBaseHeadDirections();
}

void loop() {
  int centerReading = (int) scanAndGetReading(90);
  Serial.println (centerReading);
  if(centerReading>0 && centerReading<=minimumRange){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int leftReading = scanAndGetReading(180-headTurnAngle);
  Serial.println (leftReading);
  if(leftReading>0 && leftReading<=robotWidth){
    obstacleTooCloseEmergencyStop();
    return;
  }
  int rightReading = scanAndGetReading(headTurnAngle);
  Serial.println (rightReading);
  Serial.println ("===============");
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
  int rightDecissionValue = rightReading-centerReading;
  if((leftReading!=960)&&(leftDecissionValue>minDiffForDecissionChange)&&(leftDecissionValue>=rightDecissionValue)){
    //take left turn
    base.stopAllMotion();
    base.rotateLeft(baseMovementTime);
    return;
  }
  if((rightReading!=960)&&(rightDecissionValue>minDiffForDecissionChange)&&(rightDecissionValue>leftDecissionValue)){
    //take right turn
    base.stopAllMotion();
    base.rotateRight(baseMovementTime);
    return;
  }
  //go forward
  base.goForward();
}

void obstacleTooCloseEmergencyStop(){
  tone(speakerPin, talkFrequency, 100);
  base.moveBackward(baseMovementTime);
  if(decideOnRight()){
    base.rotateRight(baseMovementTime);
  }else{
    base.rotateLeft(baseMovementTime);
  }
}

int scanAndGetReading(int angle){
  microServo.attach(servoPin);
  delay(5);
  microServo.write(angle);
  delay(500);
  microServo.detach();
  delay(5);
  int reading = (int) ultrasonicSensor.obstacleDistance();
  return reading;
}

void checkHeadPosition(){
  scanAndGetReading(90);
}

void checkBatteryVoltage(){
  int intVoltage = voltageSensor.senseVoltage();
  for(int i=0;i<intVoltage; i++){
    tone(speakerPin, shoutFrequency, 200);
    delay(400);
  }
}

void checkBaseHeadDirections(){
  base.rotateLeft(baseMovementTime);
  delay(400);
  base.rotateRight(baseMovementTime);
  delay(400);
  scanAndGetReading(180-headTurnAngle);
  delay(400);
  scanAndGetReading(90);
  delay(400);
  base.rotateRight(baseMovementTime);
  delay(400);
  base.rotateLeft(baseMovementTime);
  delay(400);
  scanAndGetReading(headTurnAngle);
  delay(400);  
  scanAndGetReading(90);
  delay(400);
  base.goForward();
  delay(400);
  base.stopAllMotion();
  delay(400);
  base.moveBackward(baseMovementTime);
  delay(400);
}

boolean decideOnRight(){
  int randNumber = random(0, 2);
  if(randNumber <1){
    return true;
  }else{
    return false;
  }
}
