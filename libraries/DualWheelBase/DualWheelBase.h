/*
  DualWheelBase.h - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#ifndef DualWheelBase_h
#define DualWheelBase_h

#include "Arduino.h"

class DualWheelBase
{
  public:
    DualWheelBase(int leftWheelForwardPin, int leftWheelBackwardPin, int rightWheelForwardPin, int rightWheelBackwardPin);
    void goForward();
	void stop();
	void goBackward();
	void rotateRight();
	void rotateLeft();
	void turnRight();
	void turnLeft();
	void steerRight(float powerDiff);
	void steerLeft(float powerDiff);
	void setPower(float powerMultiplier);
  private:
    int _leftWheelForwardPin;
	int _leftWheelBackwardPin;
    int _rightWheelForwardPin;
	int _rightWheelBackwardPin;
	float _powerMultiplier;
};

#endif