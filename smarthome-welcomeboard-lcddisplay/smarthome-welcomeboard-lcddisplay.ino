//We always have to include the library
#include "LedControl.h"

/*
  Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
  pin 12 is connected to the DataIn
  pin 11 is connected to the CLK
  pin 10 is connected to LOAD
  We have only a single MAX72XX.
*/
LedControl lc = LedControl(12, 11, 10, 1);

/* we always wait a bit between updates of the display */
unsigned long delaytime = 1000;

void setup() {
  /*
    The MAX72XX is in power-saving mode on startup,
    we have to do a wakeup call
  */
  lc.shutdown(0, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0, 8);
  /* and clear the display */
  lc.clearDisplay(0);
}

byte a[] = {B00011000, B00100100, B01000010, B01000010, B01111110, B01000010, B01000010, B01000010};
byte k[] = {B01000010,B01000100,B01001000,B01010000,B01100000,B01010000,B01001000,B01000100};
byte s[] = {B00111110,B01000000,B01000000,B01000000,B00111100,B00000010,B00000010,B01111110};
byte h[] = {B01000010, B01000010, B01000010, B01000010, B01111110, B01000010, B01000010, B01000010};
byte r[] = {B01111000, B01000100, B01000010, B01000100, B01111000, B01001000, B01000100, B01000010};
byte love[] = {B01100110,B10011001,B10011001,B10011001,B10000001,B01000010,B00100100,B00011000};

void loop() {
  displayAlphabet(a);
  delay(delaytime);
  clearDisplay();
  displayAlphabet(k);
  delay(delaytime);
  clearDisplay();
  displayAlphabet(s);
  delay(delaytime);
  clearDisplay();
  displayAlphabet(h);
  delay(delaytime);
  clearDisplay();
  displayAlphabet(a);
  delay(delaytime);
  clearDisplay();
  displayAlphabet(r);
  delay(delaytime);
  clearDisplay();
  displayAlphabet(a);
  delay(delaytime);
  clearDisplay();
  displayAlphabet(love);
  delay(2*delaytime);
  clearDisplay();
}

void clearDisplay() {
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, 0);
  }
}

void displayAlphabet(byte Digits[]) {
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, Digits[i]);
  }
}
