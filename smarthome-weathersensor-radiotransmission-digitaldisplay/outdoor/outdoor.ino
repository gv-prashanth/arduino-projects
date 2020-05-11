#include "DHT.h"
#include <SPI.h>
#include "RF24.h"

#define DHTPIN 4
#define DHTTYPE DHT22

RF24 radio (7, 8);
//Create up to 6 pipe_addresses;  the "LL" is for LongLong type
//0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL
const uint64_t pipe_addresses[] = {0x7878787878LL};//pipe between outdoor and indoor
const int led_pin = 13;
struct smallPackage
{
  float temperature;
  float humidity;
} myPayload;

DHT dht(DHTPIN, DHTTYPE);

void setup()
{
  Serial.begin(9600);
  pinMode(led_pin, OUTPUT);
  dht.begin();
  radio.begin();
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, pipe_addresses[0]);
}

void loop()
{
  if (radio.available()) {//Means there is a request
    readRequest();
    loadPayloadWithSensorReadings();
    sendPayload();
  }
}

void sendPayload() {
  Serial.println("Transmitting...");
  digitalWrite(led_pin, HIGH); // Flash a light to show transmitting
  radio.stopListening();
  radio.openWritingPipe(pipe_addresses[0]);
  radio.write(&myPayload, sizeof(myPayload));
  radio.startListening();
  digitalWrite(led_pin, LOW);
  Serial.println("Transmission complete!");
}

void readRequest() {
  unsigned char gotChar; //used to store payload from transmit module
  while (radio.available()) {// While there is data ready
    radio.read(&gotChar, sizeof(unsigned char)); //read one byte of data and store it in gotChar variable
  }
}

void loadPayloadWithSensorReadings()
{
  myPayload.humidity = dht.readHumidity();
  myPayload.temperature = dht.readTemperature();
  Serial.print("Humidity: ");
  Serial.println(myPayload.humidity);
  Serial.print("Temperature: ");
  Serial.println(myPayload.temperature);
}
