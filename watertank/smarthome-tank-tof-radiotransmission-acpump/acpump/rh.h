#include <RH_ASK.h>

#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Not actually used but needed to compile
#endif

RH_ASK driver;

void setupTransmission() {
  if (!driver.init())
    Serial.println("init failed");
}

void fetchOverheadReading() {
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  String fullString = "";
  if (driver.recv(buf, &buflen))  // Non-blocking
  {
    //int i;
    // Message with a good checksum received, dump it.
    //driver.printBuffer("Got:", buf, buflen);
    fullString = (char*)buf;
    loadAndCacheOverheadTransmissions(fullString);
  }
}
