#include <DS3231.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h";
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define DHTPIN 8       // DHT-22 Output Pin connection
#define DHTTYPE DHT22   // DHT Type is DHT 22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
DS3231  rtc(SDA, SCL);
RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002", "00003"};
struct package
  {
    float temperature = 0;
    float humidity = 0;
    int clockHr;
    int clockMin;
    int clockSec;
  };
typedef struct package Package;

void setup() {
  Serial.begin (9600);
  rtc.begin();
  lcd.begin(20,4);
  dht.begin();
  delay(200);  // Delay so DHT-22 sensor can stabalize
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, addresses[2]);
  radio.openReadingPipe(2, addresses[1]);
}
   
void loop() {
  lcd.clear();
  radio.startListening(); 
  if ( radio.available(addresses[2]))
  {
    Package data;
    radio.read(&data, sizeof(data));
    radio.stopListening();
    Serial.println(data.temperature);
    Serial.println(data.humidity);
    printMessage("WIRELESS WEATHER",String("DATE : ")+String("SORRY"),String("TIME : ")+String(data.clockHr)+String(":")+String(data.clockMin)+String(":")+String(data.clockSec));
    printMessage("INDOOR READING",String("I.TEMP : ")+String(data.temperature)+String((char)223)+String("C"),String("I.HUMID: ")+String(data.humidity)+String(" %"));
  }
  if (radio.available())
  {
    Package data;
    radio.read(&data, sizeof(data));
    radio.stopListening();
    Serial.println(data.temperature);
    Serial.println(data.humidity);
    printMessage("OUTDOOR READING",String("O.TEMP : ")+String(data.temperature)+String((char)223)+String("C"),String("O.HUMID: ")+String(data.humidity)+String(" %"));
  }  
}

void printMessage(String header, String row1, String row2){
  //cls
  lcd.clear();

  //Print welcome
  lcd.setCursor(2,0);
  lcd.print(header);
  
  // Display outdoor Temperature on third line
  lcd.setCursor(0,2);
  lcd.print(row1);
  
  delay(100);
  
  // Display outdoor Humidity on bottom line
  lcd.setCursor(0,3);
  lcd.print(row2);

  delay (3000);
}
