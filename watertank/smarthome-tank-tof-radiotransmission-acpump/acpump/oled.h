#include "OLEDDisplayFonts.h"
// For a connection via I2C using Wire include
#include <Wire.h>        // Only needed for Arduino 1.6.5 and earlier
#include "SH1106Wire.h"  //alis for `#include "SH1106Wire.h"`

// Initialize the OLED display using Wire library
SH1106Wire display(0x3c, D2, D1);

void setupDisplay() {
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void displayScreenInfo() {

  if (isConnectionWithinTreshold() && displayScreen == 1) {

    display.clear();
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 0, ("TANK:") + String(calculateTankPercentage()) + String("%"));
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 28, ("USED:") + String(calculateVolumeConsumedSoFar()) + String("L/") + String(calculateHoursConsumedSoFar()) + String("H"));
    display.display();

  } else if (isConnectionWithinTreshold() && displayScreen == 2) {

    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, ("LEVL : ") + String((int)cached_overheadTankWaterLevel) + String("cm"));
    display.drawString(0, 24, ("BATT : ") + String((0.24) + (cached_transmitterVcc)) + String("v"));
    display.drawString(0, 48, ("RSSI : ") + String(signalStrength));
    display.display();

  } else {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, " NO SIGNAL !!!");
    display.drawString(0, 24, ("BATT : ") + String(cached_transmitterVcc) + String("v"));
    display.drawString(0, 48, ("RSSI : ") + String(signalStrength));
    display.display();
  }
  unsigned long currentTime = millis();
  if (currentTime - displayChange > DISPLAY_DURATION) {
    displayScreen++;
    if (displayScreen > 2)
      displayScreen = 1;
    displayChange = currentTime;
  }
}