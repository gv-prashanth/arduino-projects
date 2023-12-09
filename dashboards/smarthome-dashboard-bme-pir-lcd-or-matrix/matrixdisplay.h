#include <MD_Parola.h>
#include <MD_MAX72xx.h>

#define MAX_DEVICES 8  // Define the number of connected MAX7219 devices
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

#define CLK_PIN 14  // Define the ESP8266 pins connected to the CLK, CS, and DATA of the dot matrix
#define DATA_PIN 13
#define CS_PIN 12

textEffect_t scrollEffect = PA_SCROLL_LEFT;
textPosition_t scrollAlign = PA_CENTER;
int scrollSpeed = 40;           // Adjust the scrolling speed
int ANIMATION_OVERHEAD = 2500;  //ms
const boolean SHOW_TIME_FREQUENTLY = true;
int INTENSITY = 0;     //0 min to 15 max
int MAXINTENSITY = 5;  //0 min to 15 max
boolean displayDayInClock = true;

//Dont touch below
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
String customText = "";  // Your global String variable
boolean prevMainMessageDisplayComplete;
unsigned long animationFinishTime;
unsigned long clockPreviousMillis, scrollPreviousMillis = 0;
boolean displayOn = true;

const unsigned long blinkDelayTime = 100;  // Delay in milliseconds
unsigned long blinkPreviousMillis = 0;
int blinkValue = 0;
bool blinkIncreasing = true;

void setDisplayMessage(String str) {
  str = replaceFirstOccurrence(str, " degree celsius", String("\xB0") + "C");
  str = replaceFirstOccurrence(str, "CALENDAR: ", "");
  str = replaceFirstOccurrence(str, "WELCOME: ", "");
  displayOn = true;
  customText = str;
  Serial.println(customText);
  P.displayClear();
  P.displayText(customText.c_str(), scrollAlign, scrollSpeed, scrollEffect, scrollEffect);
}

void turnOffDisplay() {
  displayOn = false;
  P.displayClear();
}

void setupDisplay() {
  P.begin();
  displayOn = true;
  Serial.println("Matrix display is on");
  P.displayText(DISPLAY_HEADER.c_str(), PA_CENTER, 0, 0, PA_PRINT);
  while (!P.displayAnimate()) {
    //wait till its displayed completly.
  }
  P.setIntensity(INTENSITY);
}

String getClockString() {
  String toReturn = "";
  // Use the internal clock to print the time
  if (timeStatus() == timeSet) {

    toReturn += formatHrsMins(hour(), minute(), false);

    //Date
    if (displayDayInClock) {
      String dateStr = String(monthShortStr(month())) + "," + String(day());
      dateStr.toUpperCase();
      toReturn += String(" ") + dateStr;
    }
  }
  return toReturn;
}

int blinkCountUpDown() {
  if (blinkIncreasing) {
    blinkValue++;
    if (blinkValue == MAXINTENSITY) {
      blinkIncreasing = false;
    }
  } else {
    blinkValue--;
    if (blinkValue == INTENSITY) {
      blinkIncreasing = true;
    }
  }

  return blinkValue;
}

void displayLoop(boolean dimScreen) {
  unsigned long currentTime = millis();
  if (!displayOn)
    return;
  if (dimScreen) {
    if (currentTime - blinkPreviousMillis >= ((DIM_DURATION/(MAXINTENSITY-INTENSITY))/2)) {
      blinkPreviousMillis = currentTime;

      blinkValue = blinkCountUpDown();
      // Do something with the value, for example, print it to the Serial Monitor
      //Serial.println(blinkValue);
      P.setIntensity(blinkValue);
    }
  } else
    P.setIntensity(INTENSITY);
  boolean mainMessageDisplayComplete = P.displayAnimate();
  if (SHOW_TIME_FREQUENTLY) {
    if (mainMessageDisplayComplete && !prevMainMessageDisplayComplete) {
      animationFinishTime = currentTime;
    }
    if (mainMessageDisplayComplete && (currentTime > (animationFinishTime + ANIMATION_OVERHEAD))) {
      customText = getClockString();
      P.displayText(customText.c_str(), PA_CENTER, 0, 0, PA_PRINT);
    }
    prevMainMessageDisplayComplete = mainMessageDisplayComplete;
  }
}