#include <LiquidCrystal_I2C.h>

const int SCREEN_WIDTH = 20;      //characters
const int SCREEN_HEIGHT = 4;      //rows
const int PAYLOAD_START_ROW = 2;  //index. Starts from 0.

LiquidCrystal_I2C lcd(0x27, SCREEN_WIDTH, SCREEN_HEIGHT);  // Set the LCD I2C address

void setDisplayMessage(String str) {
  str = replaceFirstOccurrence(str, " degree celsius", String((char)223) + "C");

  Serial.println(str);

  lcd.clear();
  lcd.backlight();  // Turn on the backlight

  //Print header
  lcd.setCursor((int)((SCREEN_WIDTH - DISPLAY_HEADER.length()) / 2), 0);
  lcd.print(DISPLAY_HEADER);

  // Ensure the str doesn't exceed the line limit
  if (str.length() > (SCREEN_HEIGHT - PAYLOAD_START_ROW) * SCREEN_WIDTH)
    str = str.substring(0, (SCREEN_HEIGHT - PAYLOAD_START_ROW) * SCREEN_WIDTH);

  int maxLineLength = SCREEN_WIDTH;
  int startLine = PAYLOAD_START_ROW;
  int endLine = SCREEN_HEIGHT - 1;
  String currentLine = "";
  String words[50];  // Assuming a maximum of 50 words
  int wordCount = 0;

  for (int i = 0; i < str.length(); i++) {
    char currentChar = str.charAt(i);
    if (currentChar != ' ') {
      currentLine += currentChar;
    } else {
      words[wordCount] = currentLine;
      wordCount++;
      currentLine = "";
    }
  }
  words[wordCount] = currentLine;
  wordCount++;

  // Initialize the current line with the first word
  currentLine = words[0];

  for (int i = 1; i < wordCount; i++) {
    String word = words[i];
    if (currentLine.isEmpty() || currentLine.length() + word.length() + 1 <= maxLineLength) {
      if (!currentLine.isEmpty()) {
        currentLine += ' ';  // Add a space between words
      }
      currentLine += word;
    } else {
      if (startLine < endLine) {
        // Display the current line on the LCD
        lcd.setCursor(0, startLine);
        lcd.print(currentLine);
        startLine++;
        currentLine = word;
      } else {
        // Maximum lines reached; exit the loop
        break;
      }
    }
  }

  // Display any remaining content
  if (startLine <= endLine) {
    lcd.setCursor(0, startLine);
    lcd.print(currentLine);
  }
}

void turnOffDisplay() {
  lcd.clear();
  lcd.noBacklight();
}

void setupDisplay() {
  lcd.init();  // Initialize the LCD
  setDisplayMessage("Please Wait...");
}

void displayLoop(boolean dimScreen) {
  //Nothing. It will automatically keep showing.
}
