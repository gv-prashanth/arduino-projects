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

void DualWheelBase::turnRight(){
  analogWrite(_leftWheelForwardPin, 255*_powerMultiplier);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DualWheelBase::turnLeft(){
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, LOW);
  analogWrite(_rightWheelForwardPin, 255*_powerMultiplier);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DualWheelBase::steerRight(int powerDiff){
  int maxVal = 255*_powerMultiplier;
  analogWrite(_leftWheelForwardPin, maxVal);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, maxVal - (powerDiff*_powerMultiplier));
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DualWheelBase::steerLeft(int powerDiff){
  int maxVal = 255*_powerMultiplier;
  digitalWrite(_leftWheelForwardPin, maxVal - (powerDiff*_powerMultiplier));
  digitalWrite(_leftWheelBackwardPin, LOW);
  analogWrite(_rightWheelForwardPin, maxVal);
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