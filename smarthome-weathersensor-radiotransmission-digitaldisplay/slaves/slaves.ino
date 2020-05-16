#include <SPI.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h> //download from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD I2C address
RF24 radio(9, 10); // Heard that pin 10 is not adviced to be used.

//Dont touch below stuff
struct largePackage
{
  float outtemperature;
  float outhumidity;
  float intemperature;
  float inhumidity;
  char datechar[];
  char timechar[];
} myLargePayload;
unsigned long cycleStartTime;
int prevMessageIndex = 2;
//Create up to 6 pipe_addresses;  the "LL" is for LongLong type
const uint64_t pipe_addresses[] = {0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL};

void setup() {
  Serial.begin (9600);
  lcd.begin(20, 4);
  lcd.backlight();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipe_addresses[2]);
  // Start the radio listening for data
  radio.startListening();
  cycleStartTime = micros();
}
   
void loop() {
  waitForResponseAndOnceReceivedLoadLargePayloadObject();
  keepRunningTheDisplayCycle();
}

void waitForResponseAndOnceReceivedLoadLargePayloadObject() {
  boolean timeout = false;
  unsigned long started_waiting_at = micros();
  // While nothing is received
  while (!radio.available()) {
    // If waited longer than 500ms, indicate timeout and exit while loop
    if (micros() - started_waiting_at > 500000 ) {
      timeout = true;
      break;
    }
  }
  if (!timeout) {
    while (radio.available()) {// While there is data ready
      radio.read(&myLargePayload, sizeof(myLargePayload));
    }
    Serial.println("Received response from master...");
  } else {
    Serial.println("Timeout while waiting for response from outside...");
  }
}

void keepRunningTheDisplayCycle() {
  if (micros() - cycleStartTime < 1 * 3000000) {
    printMessage(0, "WIRELESS WEATHER", String("DATE : ") + String(myLargePayload.datechar), String("TIME : ")+ String(myLargePayload.timechar));
  } else if (micros() - cycleStartTime < 2 * 3000000) {
    printMessage(1, "INDOOR READING", String("TEMP : ") + String(myLargePayload.intemperature) + String((char)223) + String("C"), String("HUMID: ") + String(myLargePayload.inhumidity) + String(" %"));
  } else if (micros() - cycleStartTime < 3 * 3000000) {
    printMessage(2, "OUTDOOR READING", String("TEMP : ") + String(myLargePayload.outtemperature) + String((char)223) + String("C"), String("HUMID: ") + String(myLargePayload.outhumidity) + String(" %"));
  } else{
    cycleStartTime = micros();
  }
}

void printMessage(int messageIndex, String header, String row1, String row2) {
  if (messageIndex != prevMessageIndex) {
    //cls
    lcd.clear();
    lcd.home();
    //Print welcome
    lcd.setCursor(2, 0);
    lcd.print(header);

    // Display Temperature on third line
    lcd.setCursor(0, 2);
    lcd.print(row1);

    // Display Humidity on bottom line
    lcd.setCursor(0, 3);
    lcd.print(row2);

    prevMessageIndex = messageIndex;
  }
}
