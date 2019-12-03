/*
  AnalogBase.h - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#ifndef AnalogBase_h
#define AnalogBase_h

#include "Arduino.h"

class AnalogBase
{
  public:
    AnalogBase(int leftWheelForwardPin, int leftWheelBackwardPin, int rightWheelForwardPin, int rightWheelBackwardPin);
    void goForward(float leftPowerMultiplier, float rightPowerMultiplier);
	void stopAllMotion();
	void goBackward(float leftPowerMultiplier, float rightPowerMultiplier);
	void rotateRight(float multiplier);
	void rotateLeft(float multiplier);
  private:
    int _leftWheelForwardPin;
	int _leftWheelBackwardPin;
    int _rightWheelForwardPin;
	int _rightWheelBackwardPin;
	void moveLeftWheelBackward(float multiplier);
	void moveLeftWheelForward(float multiplier);
	void moveRightWheelBackward(float multiplier);
	void moveRightWheelForward(float multiplier);
};

#endif