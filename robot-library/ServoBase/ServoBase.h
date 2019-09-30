/*
  ServoBase.h - Library for base movement.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#ifndef ServoBase_h
#define ServoBase_h

#include "Arduino.h"
#include "Servo.h"

class ServoBase
{
  public:
    ServoBase(int leftWheelServoPin, int rightWheelServoPin);
    void goForward();
	void stopAllMotion();
	void moveBackward(int duration);
	void rotateRight(int duration);
	void rotateLeft(int duration);
  private:
	void moveLeftWheelBackward();
	void moveLeftWheelForward();
	void moveRightWheelBackward();
	void moveRightWheelForward();
	Servo _leftWheelServo;
	Servo _rightWheelServo;
};

#endif