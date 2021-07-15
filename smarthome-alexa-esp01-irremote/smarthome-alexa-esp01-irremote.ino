#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"
#include <IRremoteESP8266.h>
#include <IRsend.h>

#define SERIAL_BAUDRATE     115200
#define WIFI_SSID           "XXXXXX"
#define WIFI_PASS           "YYYYYY"
#define DEVICE_ONE          "AC"

const uint16_t irLed = 2;

fauxmoESP fauxmo;
IRsend irsend(irLed);

//Dont touch below stuff
volatile boolean takeActionToSwitchOnDeviceOne = false;
volatile boolean takeActionToSwitchOffDeviceOne = false;

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  
  wifiSetup();
  fauxmoSetup();

  // Add device
  fauxmo.addDevice(DEVICE_ONE);
  // Callback when a command from Alexa is received.
  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
    // Just remember not to delay too much here, this is a callback, exit as soon as possible.
    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
    if (strcmp(device_name, DEVICE_ONE) == 0) {
      if (state == HIGH)
        takeActionToSwitchOnDeviceOne = true;
      else
        takeActionToSwitchOffDeviceOne = true;
    }
  });
  irsend.begin();
}

void loop() {
  fauxmo.handle();
  // fauxmo.setState(ID_YELLOW, true, 255);
  if (takeActionToSwitchOnDeviceOne) {
    triggerDeviceOneOn();
    takeActionToSwitchOnDeviceOne = false;
  }
  if (takeActionToSwitchOffDeviceOne) {
    triggerDeviceOneOff();
    takeActionToSwitchOffDeviceOne = false;
  }
}

void triggerDeviceOneOn() {
  Serial.println("DEVICE_ONE SWITCHING ON");
  irsend.sendLG(0x8800707);
}

void triggerDeviceOneOff() {
  Serial.println("DEVICE_ONE SWITCHING OFF");
  irsend.sendLG(0x88C0051);
}

void wifiSetup() {
  WiFi.mode(WIFI_STA);
  Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  // Connected!
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void fauxmoSetup() {
  fauxmo.createServer(true); // not needed, this is the default value
  fauxmo.setPort(80); // This is required for gen3 devices
  fauxmo.enable(true);
}
