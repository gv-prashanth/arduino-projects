#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Not actually used but needed to compile
#endif
RH_ASK driver;

void setupTransmission() {
#ifdef RH_HAVE_SERIAL
  Serial.println("\r\n\r\n");
#endif
  if (!driver.init())
#ifdef RH_HAVE_SERIAL
    Serial.println("init failed");
#else
    ;
#endif
}

void transmitUsingTransmitter(String totalMessage) {
  const char *msg = totalMessage.c_str();
  for (int i = 0; i < RETRY_ATTEMPTS; i++) {
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    //Serial.print(distance); Serial.println(" cm");delay(100);
  }
}