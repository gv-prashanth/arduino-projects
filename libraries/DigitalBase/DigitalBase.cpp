/*
  DigitalBase.cpp - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "DigitalBase.h"

DigitalBase::DigitalBase(int leftWheelForwardPin, int leftWheelBackwardPin, int rightWheelForwardPin, int rightWheelBackwardPin)
{
  pinMode(leftWheelForwardPin, OUTPUT);
  pinMode(leftWheelBackwardPin, OUTPUT);
  pinMode(rightWheelForwardPin, OUTPUT);
  pinMode(rightWheelBackwardPin, OUTPUT);
  _leftWheelForwardPin = leftWheelForwardPin;
  _leftWheelBackwardPin = leftWheelBackwardPin;
  _rightWheelForwardPin = rightWheelForwardPin;
  _rightWheelBackwardPin = rightWheelBackwardPin;
}

void DigitalBase::goForward(){
  moveLeftWheelForward();
  moveRightWheelForward();
}

void DigitalBase::moveBackward(int duration){
  stopAllMotion();
  moveLeftWheelBackward();
  moveRightWheelBackward();
  delay(duration);
  stopAllMotion();
}

void DigitalBase::stopAllMotion(){
  delay(5);
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, LOW);
  delay(5);
}

void DigitalBase::rotateRight(int duration){
  stopAllMotion();
  moveLeftWheelForward();
  moveRightWheelBackward();
  delay(duration);
  stopAllMotion();
}

void DigitalBase::rotateLeft(int duration){
  stopAllMotion();
  moveRightWheelForward();
  moveLeftWheelBackward();
  delay(duration);
  stopAllMotion();
}

void DigitalBase::turnRight(int duration){
  stopAllMotion();
  moveLeftWheelForward();
  delay(duration);
  stopAllMotion();
}

void DigitalBase::turnLeft(int duration){
  stopAllMotion();
  moveRightWheelForward();
  delay(duration);
  stopAllMotion();
}

void DigitalBase::moveLeftWheelBackward(){
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, HIGH);
}

void DigitalBase::moveLeftWheelForward(){
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_leftWheelForwardPin, HIGH);
}

void DigitalBase::moveRightWheelBackward(){
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, HIGH);
}

void DigitalBase::moveRightWheelForward(){
  digitalWrite(_rightWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, HIGH);
}
