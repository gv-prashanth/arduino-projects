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
    int clockDay;
    int clockMonth;
    int clockYear;
  };
typedef struct package Package;

void setup() {
  Serial.begin(9600);
  rtc.begin();
  lcd.begin(20,4);
  dht.begin();
  delay(200);  // Delay so DHT-22 sensor can stabalize
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, addresses[1]);
  radio.openWritingPipe(addresses[2]);      // 00001
}
   
void loop() {
  printMessage("WIRELESS WEATHER",String("DATE : ")+String(rtc.getDateStr()),String("TIME : ")+String(rtc.getTimeStr()));
  printMessage("INDOOR READING",String("TEMP : ")+String(dht.readTemperature())+String((char)223)+String("C"),String("HUMID: ")+String(dht.readHumidity())+String(" %"));
  radio.startListening();
  if (radio.available(addresses[1]))
  {
    Package receivedData;
    radio.read(&receivedData, sizeof(receivedData));
    radio.stopListening();
    printMessage("OUTDOOR READING",String("TEMP : ")+String(receivedData.temperature)+String((char)223)+String("C"),String("HUMID: ")+String(receivedData.humidity)+String(" %"));
    radio.startListening();
  }
  radio.stopListening();
  Package sendingData;
  sendingData.temperature = dht.readTemperature();
  sendingData.humidity = dht.readHumidity();
  sendingData.clockHr = rtc.getTime().hour;
  sendingData.clockMin = rtc.getTime().min;
  sendingData.clockSec = rtc.getTime().sec;
//  sendingData.clockDay = rtc.getTime().date;
//  sendingData.clockMonth = rtc.getTime().month;
//  sendingData.clockYear = rtc.getTime().year;
  boolean output = radio.write(&sendingData,sizeof(sendingData));
  Serial.println(output);
  delay(100);
  radio.startListening();
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
