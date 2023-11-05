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
int scrollSpeed = 50;    // Adjust the scrolling speed
String customText = "";  // Your global String variable

void setDisplayMessage(String str) {
  customText = "      " + str;
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
