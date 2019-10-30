/*
  MorseCode.h - Library for base movement.
  Created by Vader, October 30, 2019.
  Released into the public domain.
*/

#ifndef MorseCode_h
#define MorseCode_h

#include "Arduino.h"

class MorseCode
{
  public:
    MorseCode(int speakerPin, int talkFrequency, int morseUnit);
    void play(String stringToMorseCode);
  private:
	int _speakerPin;
	int _morseUnit; //unit of morse
	int _morseSymbolGapLen; //gap between symbols within a letter
	int _morseDotLen; // length of the morse code 'dot'
	int _morseDashLen; // length of the morse code 'dash'
	int _morseLetterGapLen; //length of gap betwen letters
	int _morseWordsGapLen; //length of gap betwen letters
	int _talkFrequency;//frequency in Hz
	void morseDot();
	void morseDash();
	void morseChar(char tmpChar);
};

#endif