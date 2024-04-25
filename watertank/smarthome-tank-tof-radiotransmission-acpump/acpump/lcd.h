#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setupDisplay(){
    lcd.begin(16, 2);
}

void displayScreenInfo() {
  if (isConnectionWithinTreshold() && displayScreen == 1) {
    lcd.setCursor(0, 0);
    lcd.print(String("Capacity: ") + String(calculateTankPercentage()) + String("%          "));
    lcd.setCursor(0, 1);
    lcd.print(String("Usage: ") + String(calculateVolumeConsumedSoFar()) + String("L/") + String(calculateHoursConsumedSoFar()) + String("H          "));
  } else if (isConnectionWithinTreshold() && displayScreen == 2) {
    lcd.setCursor(0, 0);
    lcd.print(String("Level: ") + String((int)cached_overheadTankWaterLevel) + String("cm          "));
    lcd.setCursor(0, 1);
    lcd.print(String("Battery: ") + String(cached_transmitterVcc) + String("v          "));
  } else {
    lcd.setCursor(0, 0);
    lcd.print(String("NO SIGNAL!!!"));
    lcd.setCursor(0, 1);
    lcd.print(String("Battery: ") + String(cached_transmitterVcc) + String("v          "));
  }
  unsigned long currentTime = millis();
  if (currentTime - displayChange > DISPLAY_DURATION) {
    displayScreen++;
    if (displayScreen > 2)
      displayScreen = 1;
    displayChange = currentTime;
  }
}