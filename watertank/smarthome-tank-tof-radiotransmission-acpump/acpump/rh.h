#include <RH_ASK.h>

#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Not actually used but needed to compile
#endif

RH_ASK driver;

void setupTransmission() {
  if (!driver.init())
    Serial.println("init failed");
}

void loadAndCacheOverheadTransmissions() {
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  unsigned long currentTime = millis();
  if (driver.recv(buf, &buflen))  // Non-blocking
  {
    //int i;
    // Message with a good checksum received, dump it.
    //driver.printBuffer("Got:", buf, buflen);
    String fullString = (char*)buf;
    int index = fullString.indexOf(',');
    String respString = fullString.substring(0, index);
    String vccTankString = fullString.substring(index + 1);
    index = vccTankString.indexOf(',');
    String vccString = vccTankString.substring(0, index);
    String tankString = vccTankString.substring(index + 1);
    //Check for tank String and then resume
    if(tankString==TANK_NAME) {
      Serial.println("vccString: "+vccString);
      Serial.println("tankString: "+tankString);
      Serial.println("Received Message from "+tankString+". And will be processed since receiver is configured to "+TANK_NAME);
    }else {
      return;
    }

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
