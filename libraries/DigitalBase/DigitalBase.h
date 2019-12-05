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
	void stop();
	void goBackward();
	void rotateRight();
	void rotateLeft();
  private:
    int _leftWheelForwardPin;
	int _leftWheelBackwardPin;
    int _rightWheelForwardPin;
	int _rightWheelBackwardPin;
};

#endif