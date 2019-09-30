#include <UltrasonicSensor.h>
#include <ServoBase.h>

//Pin Configuration
const int leftWheelServoPin = 5;//5
const int rightWheelServoPin = 10;//10
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;

//functional Configuration
const int minimumRange = 25;//cm
const int baseMovementTime = 1000;//milli seconds
const int robotWidth = 12;//cm

//Dont touch below stuff
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
ServoBase base(leftWheelServoPin, rightWheelServoPin);

void setup() {
  Serial.begin (9600);
  checkBaseHeadDirections();
}

void loop() {
  int centerReading = (int) scanAndGetReading();
  if(centerReading>0 && centerReading<=minimumRange){
    obstacleTooCloseEmergencyStop();
    return;
  }
  //go forward
  base.goForward();
}

void obstacleTooCloseEmergencyStop(){
  base.moveBackward(baseMovementTime);
  if(decideOnRight()){
    base.rotateRight(baseMovementTime);
  }else{
    base.rotateLeft(baseMovementTime);
  }
}

int scanAndGetReading(){
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
