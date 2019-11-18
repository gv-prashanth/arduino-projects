/*
  VoltageSensor.cpp - Library for Sensing voltage.
  Created by Vader, December 31, 2015.
  Released into the public domain.
  Use http://www.ohmslawcalculator.com/voltage-divider-calculator to calculate resistor values
*/

#include "Arduino.h"
#include "VoltageSensor.h"

VoltageSensor::VoltageSensor(int pin, float smallR, float bigR, float offset)
{
  _pin = pin;
  _smallR = smallR;
  _bigR = bigR;
  _offset = offset;
}

float VoltageSensor::senseVoltage(){
  const int NUM_SAMPLES = 10;
  float value = 0;
  for(int i=0;i<NUM_SAMPLES;i++){
    value += analogRead(_pin);
  }
  float avgValue = (float)value / (float)NUM_SAMPLES;
  float vOUT = (avgValue * 5.0) / 1024.0;
  float vIN = vOUT / (_smallR/(_bigR + _smallR));
  return vIN + _offset;
}