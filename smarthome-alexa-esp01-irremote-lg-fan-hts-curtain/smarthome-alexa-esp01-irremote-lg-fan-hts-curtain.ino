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

#define SERIAL_BAUDRATE 74880
#define WIFI_SSID "GTS"
#define WIFI_PASS "0607252609"
#define DEVICE_ONE "HALLAC"
#define DEVICE_TWO "FAN"
#define DEVICE_THREE "DECODER"
#define DEVICE_FOUR "BACKCURTAIN"

const int MIN_TEMPERATURE = 18;
const int MAX_TEMPERATURE = 30;
const int MIN_SOUND = 1;
const int MAX_SOUND = 60;
const int MIN_SPEED = 1;
const int MAX_SPEED = 5;
const uint16_t irLed = 3;
const int step = 2, dir = 0, Enable = 1;
const String DROID_ID = "C3PO";


//Dont touch below stuff
fauxmoESP fauxmo;
IRsend irSend(irLed);
volatile boolean takeActionToSwitchOnDeviceOne = false;
volatile boolean takeActionToSwitchOffDeviceOne = false;
volatile int deviceValueOne = 255;
volatile boolean takeActionToSwitchOnDeviceTwo = false;
volatile boolean takeActionToSwitchOffDeviceTwo = false;
volatile int deviceValueTwo = 255;
volatile boolean takeActionToSwitchOnDeviceThree = false;
volatile boolean takeActionToSwitchOffDeviceThree = false;
volatile int deviceValueThree = 255;
volatile boolean takeActionToSwitchOnDeviceFour = false;
volatile boolean takeActionToSwitchOffDeviceFour = false;
boolean curtainIsOpen = false;

void setup() {
  //NOTE ENABLING SERIAL MONITOR WILL CRASH THE APPLICATION
  //Serial.begin(SERIAL_BAUDRATE);

  pinMode(dir, OUTPUT);        // Stepper Motor Dir
  pinMode(step, OUTPUT);       // Stepper Motor Step
  pinMode(Enable, OUTPUT);     // Stepper Motor Enable
  digitalWrite(Enable, HIGH);  //Disable Stepper Motor

  wifiSetup();
  fauxmoSetup();

  // Add device
  fauxmo.addDevice(DEVICE_ONE);
  fauxmo.addDevice(DEVICE_TWO);
  fauxmo.addDevice(DEVICE_THREE);
  fauxmo.addDevice(DEVICE_FOUR);

  // Callback when a command from Alexa is received.
  fauxmo.onSetState([](unsigned char device_id, const char* device_name, bool state, unsigned char value) {
    // Just remember not to delay too much here, this is a callback, exit as soon as possible.
    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
    if (strcmp(device_name, DEVICE_ONE) == 0) {
      if (state == HIGH) {
        takeActionToSwitchOnDeviceOne = true;
      } else {
        takeActionToSwitchOffDeviceOne = true;
      }
      deviceValueOne = (int)value;
    }

    if (strcmp(device_name, DEVICE_TWO) == 0) {
      if (state == HIGH) {
        takeActionToSwitchOnDeviceTwo = true;
      } else {
        takeActionToSwitchOffDeviceTwo = true;
      }
      deviceValueTwo = (int)value;
    }

    if (strcmp(device_name, DEVICE_THREE) == 0) {
      if (state == HIGH) {
        takeActionToSwitchOnDeviceThree = true;
      } else {
        takeActionToSwitchOffDeviceThree = true;
      }
      deviceValueThree = (int)value;
    }

    if (strcmp(device_name, DEVICE_FOUR) == 0) {
      if (state == HIGH) {
        takeActionToSwitchOnDeviceFour = true;
      } else {
        takeActionToSwitchOffDeviceFour = true;
      }
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

  if (takeActionToSwitchOnDeviceTwo) {
    triggerDeviceTwoOn();
    takeActionToSwitchOnDeviceTwo = false;
  }
  if (takeActionToSwitchOffDeviceTwo) {
    triggerDeviceTwoOff();
    takeActionToSwitchOffDeviceTwo = false;
  }

  if (takeActionToSwitchOnDeviceThree) {
    triggerDeviceThreeOn();
    takeActionToSwitchOnDeviceThree = false;
  }
  if (takeActionToSwitchOffDeviceThree) {
    triggerDeviceThreeOff();
    takeActionToSwitchOffDeviceThree = false;
  }

  if (takeActionToSwitchOnDeviceFour) {
    //Toggles: Create a rountine for ON action
    triggerDeviceFourOn();
    takeActionToSwitchOnDeviceFour = false;
  }

  if (takeActionToSwitchOffDeviceFour) {
    //Toggles: There is nothing much here for toggles.
    triggerDeviceFourOff();
    takeActionToSwitchOffDeviceFour = false;
  }
}

void triggerDeviceOneOn() {
  Ac_Activate(convertValueToTemperature(deviceValueOne), 1, 0);
  sendSensorValueToAlexa("HallAC", "on%2C%20set%20to%20temperature%20" + String(convertValueToTemperature(deviceValueOne)) + "%20degree%20celsius");
}

void triggerDeviceOneOff() {
  Ac_Power_Down();
  sendSensorValueToAlexa("HallAC", "off");
}

void triggerDeviceTwoOn() {
  Fan_Activate(convertValueToSpeed(deviceValueTwo));
  sendSensorValueToAlexa("Fan", "on%2C%20set%20to%20speed%20" + String(convertValueToSpeed(deviceValueTwo)));
}

void triggerDeviceTwoOff() {
  Fan_Power_Down();
  sendSensorValueToAlexa("Fan", "%20off%2E");
}

void triggerDeviceThreeOn() {
  Decoder_ActivateArc(convertValueToSound(deviceValueThree));
}

void triggerDeviceThreeOff() {
  Decoder_ActivateOptical(convertValueToSound(deviceValueThree));
}

void triggerDeviceFourOn() {
  //Toggles: Create a rountine for ON action
  toggleCurtain();
  if (!curtainIsOpen) {
    sendSensorValueToAlexa("BackCurtain", "open");
  } else {
    sendSensorValueToAlexa("BackCurtain", "closed");
  }
}

void triggerDeviceFourOff() {
  //Not Used for toggles
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
  fauxmo.createServer(true);  // not needed, this is the default value
  fauxmo.setPort(80);         // This is required for gen3 devices
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

int convertValueToSpeed(int value) {
  if (value > 13)
    return 5;
  else if (value > 10)
    return 4;
  else if (value > 8)
    return 3;
  else if (value > 5)
    return 2;
  else
    return 1;
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
