#include "DHT.h";
#include <SPI.h>
#include <RF24.h>
#include <DS3231.h> //download from http://www.rinkydinkelectronics.com/library.php?id=73
#include <LiquidCrystal_I2C.h> //download from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/

#define DHTPIN 8       // DHT-22 Output Pin connection
#define DHTTYPE DHT22   // DHT Type is DHT 22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27);  // Set the LCD I2C address
DS3231  rtc(SDA, SCL);
RF24 radio(9, 10); // Heard that pin 10 is not adviced to be used.

//Dont touch below stuff
struct smallPackage
{
  float temperature;
  float humidity;
} mySmallPayload;
struct largePackage
{
  float outtemperature;
  float outhumidity;
  float intemperature;
  float inhumidity;
} myLargePayload;
unsigned long cycleStartTime;
int prevMessageIndex = 2;
//Create up to 6 pipe_addresses;  the "LL" is for LongLong type
const uint64_t pipe_addresses[] = {0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL};

void setup() {
  Serial.begin(9600);
  rtc.begin();
  lcd.begin(20, 4);
  dht.begin();
  radio.begin();
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipe_addresses[0]);
  radio.openReadingPipe(1, pipe_addresses[1]);
  cycleStartTime = micros();
}

void loop() {
  requestOutdoor();
  waitForResponseAndOnceReceivedLoadSmallPayloadObject();
  //loadLargePayloadObjectAndThentransmitItToSlaves();
  keepRunningTheDisplayCycle();
}

void keepRunningTheDisplayCycle() {
  if (micros() - cycleStartTime < 1 * 3000000) {
    printMessage(0, "WIRELESS WEATHER", String("DATE : ") + String(rtc.getDateStr()), String("TIME : ") + String(rtc.getTimeStr()));
  } else if (micros() - cycleStartTime < 2 * 3000000) {
    printMessage(1, "INDOOR READING", String("TEMP : ") + String(dht.readTemperature()) + String((char)223) + String("C"), String("HUMID: ") + String(dht.readHumidity()) + String(" %"));
  } else if (micros() - cycleStartTime < 3 * 3000000) {
    printMessage(2, "OUTDOOR READING", String("TEMP : ") + String(mySmallPayload.temperature) + String((char)223) + String("C"), String("HUMID: ") + String(mySmallPayload.humidity) + String(" %"));
  } else {
    cycleStartTime = micros();
  }
}

void loadLargePayloadObjectAndThentransmitItToSlaves() {
  myLargePayload.outtemperature = mySmallPayload.temperature;
  myLargePayload.outhumidity = mySmallPayload.humidity;
  myLargePayload.intemperature = dht.readTemperature();
  myLargePayload.inhumidity = dht.readHumidity();
  radio.stopListening();
  radio.openWritingPipe(pipe_addresses[1]);
  radio.write(&myLargePayload, sizeof(myLargePayload));
  radio.startListening();
  Serial.println("Transmitted readings to slaves...");
}

void requestOutdoor() {
  radio.stopListening();
  unsigned char sendChar = 1;
  radio.write(&sendChar, sizeof(unsigned char));
  radio.startListening();
  Serial.println("Requested outdoor for readings...");
}

void waitForResponseAndOnceReceivedLoadSmallPayloadObject() {
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
      radio.read(&mySmallPayload, sizeof(mySmallPayload));
    }
    Serial.println("Received response from outside...");
  } else {
    Serial.println("Timeout while waiting for response from outside...");
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
