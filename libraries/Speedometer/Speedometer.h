/*
  Speedometer.h - Library for measuring speed of a robot.
  Created by Vader, December 22, 2019.
  Released into the public domain.
*/

#ifndef Speedometer_h
#define Speedometer_h

#include "Arduino.h"

class Speedometer
{
  public:
	Speedometer(unsigned long  *lastComandedDirectionChangeTime);
	void logReading(int reading);
	float getSpeed();
  private:
	int _oldestDistance;
	unsigned long _oldestTime;
	unsigned long *_lastComandedDirectionChangeTime;
	float _currentSpeed;
};

#endif