#include <SPI.h>
#include <LoRa.h>

void setupTransmission() {
  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void transmitUsingTransmitter(String totalMessage) {
  // send packet
  LoRa.beginPacket();
  //Serial.println(totalMessage);
  LoRa.print(totalMessage);
  LoRa.endPacket();
}