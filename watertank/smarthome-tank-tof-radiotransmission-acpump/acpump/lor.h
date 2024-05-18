#include <LoRa.h>
#include <Wire.h>

#define SS 15 //Only if ESP is used. Comment out incase of Arduino
#define RST 16 //Only if ESP is used. Comment out incase of Arduino
#define DIO0 9 //Only if ESP is used. Comment out incase of Arduino

void setupTransmission() {
  LoRa.setPins(SS, RST, DIO0); //Only if ESP is used. Comment out incase of Arduino
  Wire.begin();

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa Error");
    while (1)
      ;
  }
}

void fetchOverheadReading() {
  int packetSize = LoRa.parsePacket();
  //Serial.println("attempting to read:"+packetSize);
  String fullString = "";
  if (packetSize)  // Non-blocking
  {
    //Serial.println("Receiving Data: ");
    while (LoRa.available()) {
      fullString = LoRa.readString();
    }
    //Serial.println(fullString);
    loadAndCacheOverheadTransmissions(fullString);
    signalStrength = LoRa.packetRssi();
  }
}
