#include <LoRa.h>
#include <Wire.h>

#define SS 15
#define RST 16
#define DIO0 2

void setupTransmission() {
  LoRa.setPins(SS, RST, DIO0);
  Wire.begin();

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa Error");
    while (1)
      ;
  }
}

void loadAndCacheOverheadTransmissions() {
  int packetSize = LoRa.parsePacket();
  unsigned long currentTime = millis();
  if (packetSize)  // Non-blocking
  {
    //Serial.print("Receiving Data: ");
    String fullString = "";
    while (LoRa.available()) {
      fullString = LoRa.readString();
    }
    //int i;
    // Message with a good checksum received, dump it.
    //driver.printBuffer("Got:", buf, buflen);
    
    int index = fullString.indexOf(',');
    String respString = fullString.substring(0, index);
    String vccString = fullString.substring(index + 1);
    //sometimes we are getting zero vcc from above. in such case, we used the cached value rather than setting it to zero in our receiver variable.
    if (vccString.toFloat() > 0)
      cached_transmitterVcc = vccString.toFloat() / 100;
    cached_overheadTankWaterLevel = HEIGHT_OF_TOF_SENSOR_FROM_GROUND - respString.toFloat();
    lastSuccesfulOverheadTransmissionTime = currentTime;

    cached_overheadTankWaterLevel_thisBatchAverage = ((batchCounter * cached_overheadTankWaterLevel_thisBatchAverage) / (batchCounter + 1)) + (cached_overheadTankWaterLevel / (batchCounter + 1));
    batchCounter++;
    if (currentTime - batchTimestamp > BATCH_DURATION) {
      batchTimestamp = currentTime;
      cached_overheadTankWaterLevel_prevprevBatchAverage = cached_overheadTankWaterLevel_prevBatchAverage;
      cached_overheadTankWaterLevel_prevBatchAverage = cached_overheadTankWaterLevel_thisBatchAverage;
      batchCounter = 0;
    }
    /*
    Serial.print("Tank water: ");
    Serial.print(cached_overheadTankWaterLevel);
    Serial.print(" cm. ");
    Serial.print("Prev: ");
    Serial.print(cached_overheadTankWaterLevel_prevBatchAverage);
    Serial.print(" cm. ");
    Serial.print("PrevPrev: ");
    Serial.print(cached_overheadTankWaterLevel_prevprevBatchAverage);
    Serial.println(" cm.");
    */
  }
}
