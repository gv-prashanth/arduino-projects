/*
  UltrasonicSensor.h - Library for Sensing untrasonic distance.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#ifndef UltrasonicSensor_h
#define UltrasonicSensor_h

#include "Arduino.h"

class UltrasonicSensor
{
  public:
    UltrasonicSensor(int ultraTriggerPin, int ultraEchoPin, int *reading);
    void populateReading();
  private:
    int _ultraTriggerPin;
	int _ultraEchoPin;
	int *_reading;
};

#endif