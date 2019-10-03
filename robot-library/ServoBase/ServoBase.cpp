/*
  ServoBase.cpp - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Servo.h"
#include "ServoBase.h"

ServoBase::ServoBase(int leftWheelServoPin, int rightWheelServoPin)
{
  _leftWheelServo.attach(leftWheelServoPin);
  _rightWheelServo.attach(rightWheelServoPin);
}

void ServoBase::goForward(){
  moveLeftWheelForward();
  moveRightWheelForward();
}

void ServoBase::moveBackward(int duration){
  stopAllMotion();
  moveLeftWheelBackward();
  moveRightWheelBackward();
  delay(duration);
  stopAllMotion();
}

void ServoBase::rotateRight(int duration){
  stopAllMotion();
  moveLeftWheelForward();
  moveRightWheelBackward();
  delay(duration);
  stopAllMotion();
}

void ServoBase::rotateLeft(int duration){
  stopAllMotion();
  moveRightWheelForward();
  moveLeftWheelBackward();
  delay(duration);
  stopAllMotion();
}

void ServoBase::moveLeftWheelBackward(){
  _leftWheelServo.write(0);
}

void ServoBase::moveLeftWheelForward(){
  _leftWheelServo.write(180);
}

void ServoBase::moveRightWheelBackward(){
  _rightWheelServo.write(180);
}

void ServoBase::moveRightWheelForward(){
  _rightWheelServo.write(0);
}

void ServoBase::stopAllMotion(){
  delay(5);
  _leftWheelServo.write(90);
  _rightWheelServo.write(90);
  delay(5);
}