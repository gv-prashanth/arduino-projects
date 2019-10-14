#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DigitalBase.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 11;//10
const int rightWheelBackwardPin = 10;//11
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int voltagePin = A2;

//functional Configuration
const int minimumRange = 25;//cm
const int calibratedMovementTime = 3500;//milli seconds
const int robotWidth = 20;//cm
const int talkFrequency = 2000;//frequency in Hz
const int shoutFrequency = 5000;//frquency in Hz
const int robotJamCheckTime = 15000; //milli seconds
const int semiCirclePrecission = 10;

//Dont touch below stuff
VoltageSensor voltageSensor(voltagePin);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
unsigned long lastEmergencyTime = 0;

void setup() {
  Serial.begin (9600);
  checkBatteryVoltage();
  checkBaseHeadDirections();
  lastEmergencyTime = millis()-100;
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
  base.moveBackward(calibratedMovementTime/M_PI);
  base.rotateLeft((calibratedMovementTime/360)*100);//should be mulitplied by 90 actually
  int surroundingReadings[semiCirclePrecission];
  for(int i=0;i<semiCirclePrecission; i++){
    surroundingReadings[i] = (int) getReading();
    base.rotateRight((calibratedMovementTime/360)*(180/semiCirclePrecission));
    delay(100);
  }
  int maxIndex = 0;
  int maxValue = surroundingReadings[0];
  for(int i=0;i<semiCirclePrecission; i++){
    if(surroundingReadings[i]<960 && surroundingReadings[i] > maxValue){
      maxIndex = i;
      maxValue = surroundingReadings[i];
    }
  }
  base.rotateLeft((calibratedMovementTime/360)*(180/semiCirclePrecission)*(semiCirclePrecission-maxIndex+1));
}

int getReading(){
  int reading = (int) ultrasonicSensor.obstacleDistance();
  return reading;
}

void checkBatteryVoltage(){
  int intVoltage = voltageSensor.senseVoltage();
  for(int i=0;i<intVoltage; i++){
    tone(speakerPin, talkFrequency, 200);
    delay(400);
  }
}

void checkBaseHeadDirections(){
  base.rotateLeft(calibratedMovementTime);
  delay(400);
  base.rotateRight(calibratedMovementTime);
  delay(400);
  base.goForward();
  delay(calibratedMovementTime/M_PI);
  base.stopAllMotion();
  delay(400);
  base.moveBackward(calibratedMovementTime/M_PI);
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
