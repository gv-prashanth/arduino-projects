#include <UltrasonicSensor.h>
#include <DigitalBase.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 10;//10
const int rightWheelBackwardPin = 11;//11
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;

//functional Configuration
const int minimumRange = 25;//cm
const int baseMovementTime = 500;//milli seconds
const int talkFrequency = 2000;//frequency in Hz
const int shoutFrequency = 5000;//frquency in Hz

//Dont touch below stuff
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);

void setup() {
  Serial.begin (9600);
  checkBaseHeadDirections();
}

void loop() {
  int centerReading = (int) getReading();
  Serial.println (centerReading);
  if(centerReading>0 && centerReading<=minimumRange){
    obstacleTooCloseEmergencyStop();
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

int getReading(){
  int reading = (int) ultrasonicSensor.obstacleDistance();
  return reading;
}

void checkBaseHeadDirections(){
  base.rotateLeft(baseMovementTime);
  delay(400);
  base.rotateRight(baseMovementTime);
  delay(400);
  base.rotateRight(baseMovementTime);
  delay(400);
  base.rotateLeft(baseMovementTime);
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
