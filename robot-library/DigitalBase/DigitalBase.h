/*
  DigitalBase.h - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#ifndef DigitalBase_h
#define DigitalBase_h

#include "Arduino.h"

class DigitalBase
{
  public:
    DigitalBase(int leftWheelForwardPin, int leftWheelBackwardPin, int rightWheelForwardPin, int rightWheelBackwardPin);
    void goForward();
	void stopAllMotion();
	void moveBackward(int duration);
	void rotateRight(int duration);
	void rotateLeft(int duration);
  private:
    int _leftWheelForwardPin;
	int _leftWheelBackwardPin;
    int _rightWheelForwardPin;
	int _rightWheelBackwardPin;
	void moveLeftWheelBackward();
	void moveLeftWheelForward();
	void moveRightWheelBackward();
	void moveRightWheelForward();
};

#endif