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
String DISPLAY_HEADER = "\x03 WELCOME \x03";
boolean displayOn = true;

//Dont touch below
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
String customText = "";  // Your global String variable
boolean prevMainMessageDisplayComplete;
unsigned long animationFinishTime;

void setDisplayMessage(String str) {
  str = replaceString(str, " degree celsius", String("\xB0") + "C");
  displayOn = true;
  boolean isClock = false;
  if (str.indexOf("CLOCK: ", 0) != -1) {
    isClock = true;
  }
  //str = replaceString(str, "CLOCK: ", "");
  //str = replaceString(str, "CALENDAR: ", "");
  if (isClock)
    return;  //customText = "    " + str + "   ";
  else
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

void displayScreen() {
  if(!displayOn)
    return;
  boolean mainMessageDisplayComplete = P.displayAnimate();
  if (SHOW_TIME_FREQUENTLY) {
    unsigned long currentTime = millis();
    if (mainMessageDisplayComplete && !prevMainMessageDisplayComplete) {
      animationFinishTime = currentTime;
    }
    if (mainMessageDisplayComplete && (currentTime > (animationFinishTime + ANIMATION_OVERHEAD))) {
      customText = replaceString(getSpecificSensorData("Clock").deviceReading, "at ", "");
      P.displayText(customText.c_str(), PA_CENTER, 0, 0, PA_PRINT);
    }
    prevMainMessageDisplayComplete = mainMessageDisplayComplete;
  }
}