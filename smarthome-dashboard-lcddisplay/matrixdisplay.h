#include <MD_Parola.h>
#include <MD_MAX72xx.h>

#define MAX_DEVICES 8  // Define the number of connected MAX7219 devices
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

#define CLK_PIN 14  // Define the ESP8266 pins connected to the CLK, CS, and DATA of the dot matrix
#define DATA_PIN 13
#define CS_PIN 12

MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
textEffect_t scrollEffect = PA_SCROLL_LEFT;
textPosition_t scrollAlign = PA_CENTER;
int scrollSpeed = 40;    // Adjust the scrolling speed
String customText = "";  // Your global String variable

String replaceDegreeSymbol(String inputString) {
  // Find the position of the degree symbol in the input string
  int degreeSymbolPos = inputString.indexOf("°");

  // Continue replacing the degree symbol until no more occurrences are found
  while (degreeSymbolPos != -1) {
    // Replace the degree symbol with "\xB0"
    inputString.replace("°", "\xB0");
    // Find the next occurrence of the degree symbol
    degreeSymbolPos = inputString.indexOf("°");
  }

  // Return the modified string
  return inputString;
}

String replaceForMatrix(String input, const String& search, const String& replace) {
  int index = 0;
  while ((index = input.indexOf(search, index)) != -1) {
    input = input.substring(0, index) + replace + input.substring(index + search.length());
    index += replace.length();
  }
  return input;
}

void setDisplayMessage(String str) {
  P.setIntensity(8);
  boolean isClock = false;
  if (str.indexOf("CLOCK: ", 0) != -1) {
    isClock = true;
  }
  str = replaceForMatrix(str, "CLOCK: ", "");
  str = replaceForMatrix(str, "CALENDAR: ", "");
  str = replaceDegreeSymbol(str);
  if (isClock)
    customText = "    " + str + "   ";
  else
    customText = " " + str + " ";
  Serial.println(customText);
  P.displayText(customText.c_str(), scrollAlign, scrollSpeed, scrollEffect, scrollEffect);
}

void turnOffDisplay() {
  P.setIntensity(0);
}

void setupDisplay() {
  P.begin();
  Serial.println("Matrix display is on");
  //setDisplayMessage("Please Wait...");
}

void displayScreen() {
  P.displayAnimate();
}
