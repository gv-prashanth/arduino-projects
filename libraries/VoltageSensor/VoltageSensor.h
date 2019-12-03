/*
  VoltageSensor.h - Library for Sensing voltage.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#ifndef VoltageSensor_h
#define VoltageSensor_h

#include "Arduino.h"

class VoltageSensor
{
  public:
	VoltageSensor(int pin, float smallR, float bigR);
	float senseVoltage();
  private:
	int _pin;
	float _smallR;
	float _bigR;
};

#endif