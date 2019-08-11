/*
  VoltageSensor.cpp - Library for Sensing voltage.
  Created by Vader, December 31, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "VoltageSensor.h"

VoltageSensor::VoltageSensor(int pin)
{
  _pin = pin;
}

float VoltageSensor::senseVoltage(){
  const int NUM_SAMPLES = 10;
  int sum = 0;
  for(int i=0;i<NUM_SAMPLES;i++){
    sum += analogRead(_pin);
  }
  float voltage = (((float)sum / (float)NUM_SAMPLES * 5.015) / 1024.0)* 11.132;
  return voltage;
}