#include <VoltageSensor.h>
#include <UltrasonicSensor.h>
#include <DigitalBase.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 10;//10
const int rightWheelBackwardPin = 11;//11
const int speakerPin = 4;
const int ultraTriggerPin = 7;
const int ultraEchoPin = 8;
const int voltagePin = A2;

//functional Configuration
const int minimumRange = 25;//cm
const int calibratedMovementTime = 3500;//milli seconds
int robotWidth = 20;//cm
const int robotLength = 20;//cm
const int talkFrequency = 2000;//frequency in Hz
const int morseUnit = 100; //unit of morse
const int morseDotLen = morseUnit; // length of the morse code 'dot'
const int morseDashLen = morseUnit * 3; // length of the morse code 'dash'
const int morseLetterGapLen = morseUnit * 3; //length of gap betwen letters
const int morseWordsGapLen = morseUnit * 7; //length of gap betwen letters
const int robotJamCheckTime = 15000; //milli seconds
const boolean rotateMode = true;
const float sleepVoltage = 3.5;//volts
const float wakeVoltage = 5.5;//volts. Must be greater than sleepVoltage.

//Dont touch below stuff
VoltageSensor voltageSensor(voltagePin);
UltrasonicSensor ultrasonicSensor(ultraTriggerPin, ultraEchoPin);
DigitalBase base(leftWheelForwardPin, leftWheelBackwardPin, rightWheelForwardPin, rightWheelBackwardPin);
unsigned long lastEmergencyTime = 0;

void setup() {
  Serial.begin (9600);

  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1 << 6); //enable interrupt mode

  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  //TODO: Quick hack to support both rotate and turn modes
  if (!rotateMode) {
    robotWidth = 2 * robotWidth;
  }

  //Greet
  morseString("Hello World");
  delay(3000);

  //Tell the voltage of battery
  Serial.println (voltageSensor.senseVoltage());
  int intVoltage = voltageSensor.senseVoltage();
  morseString(String(intVoltage));
  delay(3000);

  //TODO: I think i need to make a method, rather than copy paste everywhere
  //check battery
  if (voltageSensor.senseVoltage() < sleepVoltage) {
    morseString("SOS");
    sleepTillWakeVoltageIsReached();
    morseString("Battery charged");
  }

  //Body check
  checkBaseHeadDirections();
  delay(3000);

  //Load time parameters
  lastEmergencyTime = millis() - 100; //just subtracting a small time
}

void loop() {
  //check battery
  if (voltageSensor.senseVoltage() < sleepVoltage) {
    morseString("SOS");
    sleepTillWakeVoltageIsReached();
    morseString("Battery charged");
    lastEmergencyTime = millis() - 100;
    return;
  }

  //check jam
  if (checkForJam()) {
    obstacleTooCloseEmergencyStop(10 * random(0, 36));
    return;
  }

  //go left or right
  int centerReading = (int) getReading();
  if (centerReading > 0 && centerReading <= minimumRange) {
    tone(speakerPin, talkFrequency, 100);
    obstacleTooCloseEmergencyStop(90);
    return;
  }

  //go forward
  base.goForward();
}

void sleepTillWakeVoltageIsReached() {
  base.stopAllMotion();
  while (voltageSensor.senseVoltage() < wakeVoltage) {
    int sleepTime = 300; //seconds
    for (int i = 0; i < sleepTime / 8; i++) {
      //BOD DISABLE - this must be called right before the __asm__ sleep instruction
      MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
      MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
      __asm__  __volatile__("sleep");//in line assembler to go to sleep
    }
  }
}

boolean checkForJam() {
  if (abs(millis() - lastEmergencyTime) > robotJamCheckTime) {
    return true;
  }
  return false;
}

void obstacleTooCloseEmergencyStop(int angle) {
  lastEmergencyTime = millis();
  base.moveBackward((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  if (decideOnRight()) {
    if (rotateMode) {
      base.rotateRight((calibratedMovementTime / 360)*angle);
    } else {
      base.turnRight((calibratedMovementTime / 360)*angle);
    }
  } else {
    if (rotateMode) {
      base.rotateLeft((calibratedMovementTime / 360)*angle);
    } else {
      base.turnLeft((calibratedMovementTime / 360)*angle);
    }
  }
}

int getReading() {
  int reading = (int) ultrasonicSensor.obstacleDistance();
  return reading;
}

void checkBaseHeadDirections() {
  //left
  if (rotateMode) {
    base.rotateLeft(calibratedMovementTime);
  } else {
    base.turnLeft(calibratedMovementTime);
  }
  morseString("Turned left");

  //right
  if (rotateMode) {
    base.rotateRight(calibratedMovementTime);
  } else {
    base.turnRight(calibratedMovementTime);
  }
  morseString("Turned right");

  //forward
  base.goForward();
  delay((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  base.stopAllMotion();
  morseString("Went forward");

  //backward
  base.moveBackward((calibratedMovementTime / (M_PI * robotWidth))*robotLength);
  morseString("Went backward");
}

boolean decideOnRight() {
  int randNumber = random(0, 2);
  if (randNumber < 1) {
    return true;
  } else {
    return false;
  }
}

ISR(WDT_vect) {
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}// watchdog interrupt

void morseString(String stringToMorseCode)
{
  for (int i = 0; i < sizeof(stringToMorseCode) - 1; i++)
  {
    char tmpChar = stringToMorseCode[i];
    tmpChar = toLowerCase(tmpChar);
    if (tmpChar == ' ') {
      delay(morseWordsGapLen - morseLetterGapLen);
    }
    morseChar(tmpChar);
    delay(morseLetterGapLen);
  }
}

void morseDot()
{
  tone(speakerPin, talkFrequency, morseDotLen); // start playing
  delay(morseDotLen);  // hold in this position
}

void morseDash()
{
  tone(speakerPin, talkFrequency, morseDashLen);  // start playing
  delay(morseDashLen);   // hold in this position
}

void morseChar(char tmpChar)
{
  switch (tmpChar) {
    case 'a':
      morseDot(); delay(morseUnit); morseDash(); break;
    case 'b':
      morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case 'c':
      morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); break;
    case 'd':
      morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case 'e':
      morseDot(); break;
    case 'f':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); break;
    case 'g':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); break;
    case 'h':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case 'i':
      morseDot(); delay(morseUnit); morseDot(); break;
    case 'j':
      morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    case 'k':
      morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); break;
    case 'l':
      morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case 'm':
      morseDash(); delay(morseUnit); morseDash(); break;
    case 'n':
      morseDash(); delay(morseUnit); morseDot(); break;
    case 'o':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    case 'p':
      morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); break;
    case 'q':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); break;
    case 'r':
      morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); break;
    case 's':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case 't':
      morseDash(); break;
    case 'u':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); break;
    case 'v':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); break;
    case 'w':
      morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    case 'x':
      morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); break;
    case 'y':
      morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    case 'z':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case '1':
      morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    case '2':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    case '3':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    case '4':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDash(); break;
    case '5':
      morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case '6':
      morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case '7':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case '8':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); delay(morseUnit); morseDot(); break;
    case '9':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDot(); break;
    case '0':
      morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); delay(morseUnit); morseDash(); break;
    default:
      break;
  }
}
