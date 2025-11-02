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
#include <WiFiClientSecure.h>


#define SERIAL_BAUDRATE     74880
#define WIFI_SSID           "XXXXXXX"
#define WIFI_PASS           "YYYYYYY"
#define DEVICE_ONE          "Daikin"


const int MIN_TEMPERATURE = 18;
const int MAX_TEMPERATURE = 24;
const String DROID_ID = "ABCD";
const uint16_t irLed = 3;

//Dont touch below stuff
fauxmoESP fauxmo;
//IRsend irsend(irLed);
IRDaikinESP irsend(irLed);
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

  irsend.begin();
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
  Serial.println("DEVICE_ONE SWITCHING ON");
  //irsend.sendLG(0x8800707);
  irsend.on();
  irsend.setMode(kDaikinCool);
  irsend.setTemp(convertValueToTemperature(deviceValueOne));
  irsend.setFan(kDaikinFanAuto);
  irsend.setSwingVertical(false);
  irsend.setSwingHorizontal(false);
  irsend.setComfort(true);//irsend.setQuiet(false);// Powerful, Quiet, Comfort, & Econo mode being on are mutually exclusive.  Serial.println(irsend.toString());
  Serial.println(irsend.toString());
  irsend.send();
  sendSensorValueToAlexa(DEVICE_ONE, "On%20"+String(convertValueToTemperature(deviceValueOne))+"%C2%B0C");
}

void triggerDeviceOneOff() {
  Serial.println("DEVICE_ONE SWITCHING OFF");
  //irsend.sendLG(0x88C0051);
  irsend.off();
  irsend.setMode(kDaikinCool);
  irsend.setTemp(convertValueToTemperature(deviceValueOne));
  irsend.setFan(kDaikinFanAuto);
  irsend.setSwingVertical(false);
  irsend.setSwingHorizontal(false);
  irsend.setComfort(true);//irsend.setQuiet(false);// Powerful, Quiet, Comfort, & Econo mode being on are mutually exclusive.  Serial.println(irsend.toString());
  Serial.println(irsend.toString());
  irsend.send();
  sendSensorValueToAlexa(DEVICE_ONE, "Off");
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
  String encodedName = "";
  for (int i = 0; i < name.length(); i++) encodedName += (name.charAt(i) == ' ') ? "%20" : String(name.charAt(i));
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String fullUrl = "https://home-automation.vadrin.com/droid/" + DROID_ID + "/upsert/intent/" + encodedName + "/reading/" + reading;
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