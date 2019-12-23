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
	Speedometer();
	void logReading(int reading);
	void clearReadings();
	float getSpeed();
  private:
	int _oldestDistance, _latestDistance;
	unsigned long _oldestTime, _latestTime;
};

#endif