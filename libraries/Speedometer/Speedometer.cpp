/*
  Speedometer.h - Library for measuring speed of a robot.
  Created by Vader, December 22, 2019.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Speedometer.h"

Speedometer::Speedometer(unsigned long *lastComandedDirectionChangeTime){
	_oldestDistance = -1;
	_oldestTime = 0;
	_currentSpeed = -1;
	_lastComandedDirectionChangeTime = lastComandedDirectionChangeTime;
}

void Speedometer::logReading(int reading){
	if(reading < 960){
		if(_oldestTime == 0 || _oldestTime < *_lastComandedDirectionChangeTime){
			_oldestTime = millis();
			_oldestDistance = reading;
			return;
		}
		if(reading - _oldestDistance > 1){
			unsigned long currentTime = millis();
			_currentSpeed = ((reading - _oldestDistance)*1000.0)/(currentTime-_oldestTime);
			_oldestTime = currentTime;
			_oldestDistance = reading;
			return;
		}else{
			Serial.println("Inconsistent reading. Didnt travel enough yet");
		}
	}else{
		Serial.println("Inconsistent reading. Reading is ignored");
	}
}

float Speedometer::getSpeed(){
	return _currentSpeed;
}