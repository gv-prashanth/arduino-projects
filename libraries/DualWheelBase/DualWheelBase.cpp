/*
  DualWheelBase.cpp - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "DualWheelBase.h"

DualWheelBase::DualWheelBase(int leftWheelForwardPin, int leftWheelBackwardPin, int rightWheelForwardPin, int rightWheelBackwardPin)
{
  _leftWheelForwardPin = leftWheelForwardPin;
  _leftWheelBackwardPin = leftWheelBackwardPin;
  _rightWheelForwardPin = rightWheelForwardPin;
  _rightWheelBackwardPin = rightWheelBackwardPin;
  _powerMultiplier = 1.0;
  pinMode(_leftWheelForwardPin, OUTPUT);
  pinMode(_leftWheelBackwardPin, OUTPUT);
  pinMode(_rightWheelForwardPin, OUTPUT);
  pinMode(_rightWheelBackwardPin, OUTPUT);
}

void DualWheelBase::goForward(){
  analogWrite(_leftWheelForwardPin, 255*_powerMultiplier);
  digitalWrite(_leftWheelBackwardPin, LOW);
  analogWrite(_rightWheelForwardPin, 255*_powerMultiplier);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DualWheelBase::goBackward(){
  digitalWrite(_leftWheelForwardPin, LOW);
  analogWrite(_leftWheelBackwardPin, 255*_powerMultiplier);
  digitalWrite(_rightWheelForwardPin, LOW);
  analogWrite(_rightWheelBackwardPin, 255*_powerMultiplier);
}

void DualWheelBase::rotateRight(){
  analogWrite(_leftWheelForwardPin, 255*_powerMultiplier);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, LOW);
  analogWrite(_rightWheelBackwardPin, 255*_powerMultiplier);
}

void DualWheelBase::rotateLeft(){
  digitalWrite(_leftWheelForwardPin, LOW);
  analogWrite(_leftWheelBackwardPin, 255*_powerMultiplier);
  analogWrite(_rightWheelForwardPin, 255*_powerMultiplier);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DualWheelBase::goForward(float powerDiff){
  float v1 = (255 + powerDiff)*_powerMultiplier;
  float v2 = (255 - powerDiff)*_powerMultiplier;
  if(v1 >255)
	  v1 = 255;
  if(v2 >255)
	  v2 = 255;
  if(v1 <0)
	  v1 = 0;
  if(v2 <0)
	  v2 = 0;
  //If powerDiff is negative i need to steer left - v2 is more than v1
  //If powerDiff is positive i need to steer right - v1 is more than v2
  digitalWrite(_leftWheelForwardPin, v1);
  digitalWrite(_leftWheelBackwardPin, LOW);
  analogWrite(_rightWheelForwardPin, v2);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DualWheelBase::setPower(float powerMultiplier){
  if(powerMultiplier < 0){
    _powerMultiplier = 0;
  }else if(powerMultiplier > 1){
    _powerMultiplier = 1;
  }else{
    _powerMultiplier = powerMultiplier;
  }
}

void DualWheelBase::stop(){
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, LOW);
}