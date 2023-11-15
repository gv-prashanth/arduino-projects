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
int INTENSITY = 0;
boolean displayOn = true;

//Dont touch below
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
String customText = "";  // Your global String variable
boolean prevMainMessageDisplayComplete;
unsigned long animationFinishTime;
unsigned long clockPreviousMillis, scrollPreviousMillis = 0;
const long interval = 1000;

void setDisplayMessage(String str) {
  str = replaceString(str, " degree celsius", String("\xB0") + "C");
  str = replaceString(str, "CALENDAR: ", "");
  str = replaceString(str, "WELCOME: ", "");
  displayOn = true;
  customText = " " + str + " ";
  Serial.println(customText);
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
  unsigned long currentMillis = millis();
  if (currentMillis - clockPreviousMillis >= interval) {
    clockPreviousMillis = currentMillis;

    // Use the internal clock to print the time
    if (timeStatus() == timeSet) {

      String hrString = "";
      String minString = "";
      int hr = hour();
      bool isPM = hr >= 12;  // Check if it's PM
      if (hr > 12) {
        hr -= 12;  // Convert 24-hour to 12-hour format
      }
      if (hr == 0) {
        hr = 12;  // 0:00 should be 12:00 AM
      }

      if (hr < 10) {
        //Serial.print("0");
        hrString += "0";
      }

      //Serial.print(hr);
      hrString += hr;
      //Serial.print(":");

      if (minute() < 10) {
        //Serial.print("0");
        minString += "0";
      }
      //Serial.print(minute());
      minString += minute();
      //Serial.println();

      toReturn += hrString;
      if (second() % 2 == 1) {
        toReturn += ":";
      } else {
        toReturn += " ";
      }
      toReturn += minString;
    }
  }
  return toReturn;
}

void displayScreen() {
  if (!displayOn)
    return;
  boolean mainMessageDisplayComplete = P.displayAnimate();
  if (SHOW_TIME_FREQUENTLY) {
    unsigned long currentTime = millis();
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