/*
  MorseCode.cpp - Library for making morse code tones.
  Created by Vader, October 30, 2019.
  Released into the public domain.
*/

#include "Arduino.h"
#include "MorseCode.h"

MorseCode::MorseCode(int speakerPin, int talkFrequency, int morseUnit)
{
	_speakerPin = speakerPin;
	_talkFrequency = talkFrequency;
	_morseUnit = morseUnit; //unit of morse
	_morseSymbolGapLen = _morseUnit; //gap between symbols within a letter
	_morseDotLen = _morseUnit; // length of the morse code 'dot'
	_morseDashLen = _morseUnit * 3; // length of the morse code 'dash'
	_morseLetterGapLen = _morseUnit * 3; //length of gap betwen letters
	_morseWordsGapLen = _morseUnit * 7; //length of gap betwen letters
}

void MorseCode::play(String stringToMorseCode){
  for (int i = 0; i < sizeof(stringToMorseCode) - 1; i++)
  {
    char tmpChar = stringToMorseCode[i];
    tmpChar = toLowerCase(tmpChar);
    if (tmpChar == ' ') {
      delay(_morseWordsGapLen - _morseLetterGapLen);
    }
    morseChar(tmpChar);
    delay(_morseLetterGapLen);
  }
}

void MorseCode::morseDot(){
  tone(_speakerPin, _talkFrequency, _morseDotLen); // start playing
  delay(_morseDotLen);  // hold in this position
}

void MorseCode::morseDash(){
  tone(_speakerPin, _talkFrequency, _morseDashLen);  // start playing
  delay(_morseDashLen);   // hold in this position
}

void MorseCode::morseChar(char tmpChar){
  switch (tmpChar) {
    case 'a':
      morseDot(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'b':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'c':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'd':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'e':
      morseDot(); break;
    case 'f':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'g':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'h':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'i':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'j':
      morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'k':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'l':
      morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'm':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'n':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'o':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'p':
      morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); break;
    case 'q':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'r':
      morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); break;
    case 's':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case 't':
      morseDash(); break;
    case 'u':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'v':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'w':
      morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'x':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'y':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case 'z':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case '1':
      morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case '2':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case '3':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    case '4':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDash(); break;
    case '5':
      morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case '6':
      morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case '7':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case '8':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); delay(_morseSymbolGapLen); morseDot(); break;
    case '9':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDot(); break;
    case '0':
      morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); delay(_morseSymbolGapLen); morseDash(); break;
    default:
      break;
  }
}