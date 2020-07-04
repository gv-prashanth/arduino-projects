/*
  UltrasonicSensor.cpp - Library for Sensing untrasonic distance.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(int ultraTriggerPin, int ultraEchoPin)
{
  pinMode(ultraTriggerPin, OUTPUT);
  pinMode(ultraEchoPin, INPUT);
  _ultraTriggerPin = ultraTriggerPin;
  _ultraEchoPin = ultraEchoPin;
}

long UltrasonicSensor::obstacleDistance() {
  long duration, distance;
  digitalWrite(_ultraTriggerPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(_ultraTriggerPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(_ultraTriggerPin, LOW);
  duration = pulseIn(_ultraEchoPin, HIGH, 58200);
  //58200 is the cut off time after which the sensor will stop waiting for response
  distance = duration/58.2;
  delay(10);
  if(distance>0){
    return distance;
  }
  return 960;//cm
  //this is half the distance travelled by sound in 58200 microseconds
}