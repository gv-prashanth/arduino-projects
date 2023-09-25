#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"
#include <IRremoteESP8266.h>
#include <irSend.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>


#define SERIAL_BAUDRATE     74880
#define WIFI_SSID           "XXXXXXX"
#define WIFI_PASS           "YYYYYYY"
#define DEVICE_ONE          "LG BED ROOM"


const int MIN_TEMPERATURE = 18;
const int MAX_TEMPERATURE = 30;
const String DROID_ID = "C3PO";
const uint16_t irLed = 3;

//Dont touch below stuff
fauxmoESP fauxmo;
IRsend irSend(irLed);
volatile boolean takeActionToSwitchOnDeviceOne = false;
volatile boolean takeActionToSwitchOffDeviceOne = false;
volatile int deviceValueOne = 255;

void setup() {
  //NOTE ENABLING SERIAL MONITOR WILL CRASH THE APPLICATION
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
      if (state == HIGH) {
        takeActionToSwitchOnDeviceOne = true;
      } else {
        takeActionToSwitchOffDeviceOne = true;
      }
      deviceValueOne = (int) value;
    }


  });

  irSend.begin();
}

void loop() {
  fauxmo.handle();

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
  Ac_Activate(convertValueToTemperature(deviceValueOne), 1, 0);
  sendSensorValueToAlexa("BedAC", "On%2C%20set%20to%20temperature%20"+String(convertValueToTemperature(deviceValueOne))+"%20degree%20celsius");
}

void triggerDeviceOneOff() {
  Ac_Power_Down();
  sendSensorValueToAlexa("BedAC", "Off");
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

int convertValueToTemperature(int value) {
  int temp = (100.0 / 255.0) * (value);
  if (temp <= MIN_TEMPERATURE)
    temp = MIN_TEMPERATURE;
  if (temp >= MAX_TEMPERATURE)
    temp = MAX_TEMPERATURE;
  return temp;
}


void sendSensorValueToAlexa(String name, String reading) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String fullUrl = "https://home-automation.vadrin.com/droid/" + DROID_ID + "/upsert/intent/" + name + "/reading/" + reading;
  Serial.println("Requesting " + fullUrl);
  if (https.begin(client, fullUrl)) {
    int httpCode = https.GET();
    Serial.println("============== Response code: " + String(httpCode));
    if (httpCode > 0) {
      Serial.println(https.getString());
    }
    https.end();
  } else {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
}
