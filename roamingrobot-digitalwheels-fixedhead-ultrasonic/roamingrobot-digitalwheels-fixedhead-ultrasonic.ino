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
const int calibratedMovementTime = 3500;//milli seconds
int robotWidth = 20;//cm
const int robotLength = 20;//cm
const int talkFrequency = 2000;//frequency in Hz
const int shoutFrequency = 5000;//frquency in Hz
const int robotJamCheckTime = 15000; //milli seconds
const boolean rotateMode = false;

//Dont touch below stuff
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
unsigned long lastEmergencyTime = 0;

void setup() {
  //TODO: Quick hack to solve to support both rotate and turn modes
  if(!rotateMode){
    robotWidth = 2 * robotWidth;
  }
  Serial.begin (9600);
  tone(speakerPin, talkFrequency, 3000);
  delay(3000);
  checkBaseHeadDirections();
  lastEmergencyTime = millis()-100;//just subtracting a small time
}

void loop() {
  int centerReading = (int) getReading();
  Serial.println (centerReading);
  if(centerReading>0 && centerReading<=minimumRange){
    obstacleTooCloseEmergencyStop();
    return;
  }
  if(checkForJam()){
    obstacleTooCloseEmergencyStop();
    return;
  }
  //go forward
  base.goForward();
}

boolean checkForJam(){
  if(abs(millis()-lastEmergencyTime) > robotJamCheckTime){
    tone(speakerPin, talkFrequency, 500);
    return true;
  }
  return false;
}

void obstacleTooCloseEmergencyStop(){
  tone(speakerPin, shoutFrequency, 100);
  lastEmergencyTime = millis();
  base.moveBackward((calibratedMovementTime/(M_PI*robotWidth))*robotLength);
  if(decideOnRight()){
    if(rotateMode){
      base.rotateRight((calibratedMovementTime/360)*90);
    }else{
      base.turnRight((calibratedMovementTime/360)*90);
    }
  }else{
    if(rotateMode){
      base.rotateLeft((calibratedMovementTime/360)*90);
    }else{
      base.turnLeft((calibratedMovementTime/360)*90);
    }
  }
}

int getReading(){
  int reading = (int) ultrasonicSensor.obstacleDistance();
  return reading;
}

void checkBaseHeadDirections(){
  if(rotateMode){
    base.rotateLeft(calibratedMovementTime);
  }else{
    base.turnLeft(calibratedMovementTime);
  }
  delay(400);
  if(rotateMode){
    base.rotateRight(calibratedMovementTime);
  }else{
    base.turnRight(calibratedMovementTime);
  }
  delay(400);
  base.goForward();
  delay((calibratedMovementTime/(M_PI*robotWidth))*robotLength);
  base.stopAllMotion();
  delay(400);
  base.moveBackward((calibratedMovementTime/(M_PI*robotWidth))*robotLength);
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
