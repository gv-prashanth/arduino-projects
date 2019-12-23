/*
  Speedometer.h - Library for measuring speed of a robot.
  Created by Vader, December 22, 2019.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Speedometer.h"

Speedometer::Speedometer(){
	_oldestDistance = 0;
	_latestDistance = 0;
	_oldestTime = 0;
	_latestTime = 0;
}

void Speedometer::logReading(int reading){
	if(reading < 960){
		if(_oldestTime == 0){
			_oldestTime = millis();
			_oldestDistance = reading;
		}else{
			if(_latestTime != 0){
				_oldestTime = _latestTime;
				_oldestDistance = _latestDistance;	
			}
			_latestTime = millis();
			_latestDistance = reading;
		}
	}
}

void Speedometer::clearReadings(){
	_oldestDistance = 0;
	_latestDistance = 0;
	_oldestTime = 0;
	_latestTime = 0;
}

float Speedometer::getSpeed(){
	return ((_latestDistance - _oldestDistance)*1000)/(_latestTime - _oldestTime);
}