/*
  DigitalBase.cpp - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "DigitalBase.h"

DigitalBase::DigitalBase(int leftWheelForwardPin, int leftWheelBackwardPin, int rightWheelForwardPin, int rightWheelBackwardPin)
{
  _leftWheelForwardPin = leftWheelForwardPin;
  _leftWheelBackwardPin = leftWheelBackwardPin;
  _rightWheelForwardPin = rightWheelForwardPin;
  _rightWheelBackwardPin = rightWheelBackwardPin;
  pinMode(_leftWheelForwardPin, OUTPUT);
  pinMode(_leftWheelBackwardPin, OUTPUT);
  pinMode(_rightWheelForwardPin, OUTPUT);
  pinMode(_rightWheelBackwardPin, OUTPUT);
}

void DigitalBase::goForward(){
  digitalWrite(_leftWheelForwardPin, HIGH);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, HIGH);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DigitalBase::goBackward(){
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, HIGH);
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, HIGH);
}

void DigitalBase::rotateRight(){
  digitalWrite(_leftWheelForwardPin, HIGH);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, HIGH);
}

void DigitalBase::rotateLeft(){
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, HIGH);
  digitalWrite(_rightWheelForwardPin, HIGH);
  digitalWrite(_rightWheelBackwardPin, LOW);
}

void DigitalBase::stop(){
  digitalWrite(_leftWheelForwardPin, LOW);
  digitalWrite(_leftWheelBackwardPin, LOW);
  digitalWrite(_rightWheelForwardPin, LOW);
  digitalWrite(_rightWheelBackwardPin, LOW);
}