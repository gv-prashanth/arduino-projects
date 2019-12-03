/*
  AnalogBase.cpp - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "AnalogBase.h"

AnalogBase::AnalogBase(int leftWheelForwardPin, int leftWheelBackwardPin, int rightWheelForwardPin, int rightWheelBackwardPin)
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

void AnalogBase::goForward(float leftPowerMulitplier, float rightPowerMultiplier){
  stopAllMotion();
  moveLeftWheelForward(leftPowerMulitplier);
  moveRightWheelForward(rightPowerMultiplier);
}

void AnalogBase::goBackward(float leftPowerMulitplier, float rightPowerMultiplier){
  stopAllMotion();
  moveLeftWheelBackward(leftPowerMulitplier);
  moveRightWheelBackward(rightPowerMultiplier);
}

void AnalogBase::stopAllMotion(){
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void AnalogBase::rotateRight(float multiplier){
  stopAllMotion();
  moveLeftWheelForward(multiplier);
}

void AnalogBase::rotateLeft(float multiplier){
  stopAllMotion();
  moveRightWheelForward(multiplier);
}

void AnalogBase::moveLeftWheelBackward(float multiplier){
  digitalWrite(_leftWheelForwardPin, LOW);
  analogWrite(_leftWheelBackwardPin, 255*multiplier);
}

void AnalogBase::moveLeftWheelForward(float multiplier){
  digitalWrite(_leftWheelBackwardPin, LOW);
  analogWrite(_leftWheelForwardPin, 255*multiplier);
}

void AnalogBase::moveRightWheelBackward(float multiplier){
  digitalWrite(_rightWheelForwardPin, LOW);
  analogWrite(_rightWheelBackwardPin, 255*multiplier);
}

void AnalogBase::moveRightWheelForward(float multiplier){
  digitalWrite(_rightWheelBackwardPin, LOW);
  analogWrite(_rightWheelForwardPin, 255*multiplier);
}