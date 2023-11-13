#include <WiFiClient.h>
#include <Arduino.h>
#define USE_SERIAL_2004_LCD   //#define USE_SERIAL_1602_LCD
#include "LCDBigNumbers.hpp"  // Include sources for LCD big number generation

const int SCREEN_WIDTH = 20;  //characters
const int SCREEN_HEIGHT = 4;  //rows
String DISPLAY_HEADER = "WELCOME";
const long scrollSpeed = 400;

LiquidCrystal_I2C lcd(0x27, SCREEN_WIDTH, SCREEN_HEIGHT);                             // Set the LCD I2C address
LCDBigNumbers ThreeLineNumbersLCD(&lcd, BIG_NUMBERS_FONT_3_COLUMN_3_ROWS_VARIANT_1);  // Use 3x3 numbers
unsigned long clockPreviousMillis, scrollPreviousMillis = 0;
const long interval = 1000;
int scrollPosition = 0;
String longText = "";
boolean displayOn = true;

void printTextOnLastRowAsScroll() {
  int displayWidth = LCD_COLUMNS;
  if (longText.length() <= displayWidth) {
    // Text fits within the display, no need to scroll
    lcd.setCursor((displayWidth - longText.length()) / 2, 3);
    lcd.print(longText);
  } else {
    // Text needs to scroll
    if (scrollPosition <= longText.length() - displayWidth) {
      lcd.setCursor(0, 3);
      lcd.print(longText.substring(scrollPosition, scrollPosition + displayWidth));
      scrollPosition++;
    } else {
      // End of text reached, stop scrolling
      lcd.setCursor(0, 3);
      lcd.print(longText.substring(longText.length() - displayWidth));
    }
  }
}

void printClockOnFirstThreeRowsEverySecond() {
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

      ThreeLineNumbersLCD.setBigNumberCursor(1);
      ThreeLineNumbersLCD.print(hrString);
      if (second() % 2 == 1) {
        ThreeLineNumbersLCD.print(':');
      } else {
        ThreeLineNumbersLCD.print(ONE_COLUMN_SPACE_CHARACTER);
      }
      ThreeLineNumbersLCD.setBigNumberCursor(11);
      ThreeLineNumbersLCD.print(minString);
    }
  }
}

void setDisplayMessage(String str) {
  str = replaceString(str, " degree celsius", String((char)223) + "C");
  Serial.println(str);
  displayOn = true;
  longText = str;
  scrollPosition = 0;
  lcd.setCursor(0, 3);
  lcd.print("                    ");
}

void turnOffDisplay() {
  displayOn = false;
  lcd.clear();
  lcd.noBacklight();
}

void setupDisplay() {
  displayOn = true;
  lcd.init();
  lcd.clear();
  lcd.backlight();
  ThreeLineNumbersLCD.begin();  // Creates custom character used for generating big numbers
  setDisplayMessage(DISPLAY_HEADER);
  printTextOnLastRowAsScroll();
}

void displayScreen() {
  if (!displayOn)
    return;
  unsigned long currentMillis = millis();
  lcd.backlight();
  printClockOnFirstThreeRowsEverySecond();
  if (currentMillis - scrollPreviousMillis >= scrollSpeed) {
    scrollPreviousMillis = currentMillis;
    printTextOnLastRowAsScroll();
  }
}