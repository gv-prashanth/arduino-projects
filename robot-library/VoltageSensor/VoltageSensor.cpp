/*
  VoltageSensor.cpp - Library for Sensing voltage.
  Created by Vader, December 31, 2015.
  Released into the public domain.
  Use http://www.ohmslawcalculator.com/voltage-divider-calculator to calculate resistor values
*/

#include "Arduino.h"
#include "VoltageSensor.h"

VoltageSensor::VoltageSensor(int pin, float smallR, float bigR)
{
  _pin = pin;
  _smallR = smallR;
  _bigR = bigR;
}

float VoltageSensor::senseVoltage(){
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  if(_pin==-1)
	//Means there is no dedicated sensor pin connected, so return vcc voltage
	return (((float)result)/1000.0);
  else {
	// Read the sense pin voltage using the previously calculated vcc voltage
	float avgValue = analogRead(_pin);
	float vOUT = (avgValue * (((float)result)/1000.0)) / 1024.0;
	float vIN = vOUT / (_smallR/(_bigR + _smallR));
	return vIN;
  }
}