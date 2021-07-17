#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Daikin.h>

#define SERIAL_BAUDRATE     115200
#define WIFI_SSID           "XXXXXX"
#define WIFI_PASS           "YYYYYY"
#define DEVICE_ONE          "AC"
const int MIN_TEMPERATURE = 18;
const int MAX_TEMPERATURE = 32;
const uint16_t irLed = 2;

//Dont touch below stuff
fauxmoESP fauxmo;
//IRsend irsend(irLed);
IRDaikinESP irsend(irLed);

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
      int temp = convertValueToTemperature(value);
      if (state == HIGH) {
        triggerDeviceOneOn(temp);
      } else
        triggerDeviceOneOff(temp);
    }
  });
  irsend.begin();
}

void loop() {
  fauxmo.handle();
  // fauxmo.setState(ID_YELLOW, true, 255);
}

void triggerDeviceOneOn(int temp) {
  Serial.println("DEVICE_ONE SWITCHING ON");
  //irsend.sendLG(0x8800707);
  irsend.on();
  irsend.setMode(kDaikinCool);
  irsend.setTemp(temp);
  irsend.setFan(kDaikinFanAuto);
  irsend.setSwingVertical(false);
  irsend.setSwingHorizontal(false);
  irsend.setQuiet(false);
  irsend.setPowerful(false);
  Serial.println(irsend.toString());
  irsend.send();
}

void triggerDeviceOneOff(int temp) {
  Serial.println("DEVICE_ONE SWITCHING OFF");
  //irsend.sendLG(0x88C0051);
  irsend.off();
  irsend.setMode(kDaikinCool);
  irsend.setTemp(temp);
  irsend.setFan(kDaikinFanAuto);
  irsend.setSwingVertical(false);
  irsend.setSwingHorizontal(false);
  irsend.setQuiet(false);
  irsend.setPowerful(false);
  Serial.println(irsend.toString());
  irsend.send();
}

int convertValueToTemperature(unsigned char value) {
  int temp = (100.0 / 255.0) * ((int)value);
  if (temp < MIN_TEMPERATURE)
    temp = MIN_TEMPERATURE;
  if (temp > MAX_TEMPERATURE)
    temp = MAX_TEMPERATURE;
  return temp;
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
