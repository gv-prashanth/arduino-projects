#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

// -------------------------------CONFIGURATIONS--------------------------------
#define WIFI_SSID "XXX"
#define WIFI_PASS "YYY"
const String DROID_ID = "ZZZ";
#define SERIAL_BAUDRATE 115200
#define CURTAIN "CURTAIN"
String CURTAINKEY = "Curtain";
const int step = 2, dir = 0, Enable = 1, irSensorPin = 3;

// -----------------------------------------------------------------------------


// -------------------------------DONT TOUCH------------------------------------
fauxmoESP fauxmo;
volatile boolean takeActionToSwitchOnCurtain, takeActionToSwitchOffCurtain;
volatile int curtainValue = 255;
volatile boolean prevLoop_isCurtainOpen, expected_isCurtainOpen;
// -----------------------------------------------------------------------------

void setup() {

  // Init serial port and clean garbage
  //Serial.begin(SERIAL_BAUDRATE); Serial.println(); Serial.println();

  // PINS
  pinMode(dir, OUTPUT);     // Stepper Motor Dir
  pinMode(step, OUTPUT);    // Stepper Motor Step
  pinMode(Enable, OUTPUT);  // Stepper Motor Enable
  pinMode(irSensorPin, INPUT);
  digitalWrite(Enable, HIGH);  //Disable Stepper Motor

  // Wifi
  wifiSetup();

  // fauxmo
  fauxmoSetup();

  // load the prev and expected status
  prevLoop_isCurtainOpen = !isCurtainOpen();
  expected_isCurtainOpen = isCurtainOpen();
}

void loop() {
  fauxmoLoop();
  checkAndTakeAction();
  checkForErrorAndSendToAlexa();
  checkForChangeAndSendToAlexa();
}

void wifiSetup() {

  // Set WIFI module to STA mode
  WiFi.mode(WIFI_STA);

  // Connect
  Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Connected!
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
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

void openCurtain() {
  if (isCurtainOpen())
    return;
  Serial.println("CURTAIN OPENING");
  // Put the motor in forward:
  delay(500);                 // delay for Controller startup
  digitalWrite(dir, HIGH);    // Turn MOTOR FORWARD / OPEN CURTAIN
  digitalWrite(Enable, LOW);  // Enable Stepper Motor

  for (double i = 0; i < 2300; i++)  //6600 steps in one direction
  {
    digitalWrite(step, HIGH);
    delay(1);
    digitalWrite(step, LOW);
    delay(1);
  }
  digitalWrite(Enable, HIGH);  //Disable Stepper Motor
}

void closeCurtain() {
  if (!isCurtainOpen())
    return;
  Serial.println("CURTAIN CLOSING");
  // Put the motor in reverse:
  delay(500);                 // delay for Controller startup
  digitalWrite(dir, LOW);     // Turn MOTOR REVBERSE / CLOSE CURTAIN
  digitalWrite(Enable, LOW);  // Enable Stepper Motor

  for (double i = 0; i < 2300; i++)  //6400 steps in one direction
  {
    digitalWrite(step, HIGH);
    delay(1);
    //delayMicroseconds(1000);
    digitalWrite(step, LOW);
    delay(1);
    //delayMicroseconds(1000);
  }
  digitalWrite(Enable, HIGH);  //Disable Stepper Motor
}

boolean isCurtainOpen() {
  int sensorValue = digitalRead(irSensorPin);
  if (sensorValue == HIGH) {
    Serial.println("IR Proximity Sensor ON");
    return false;
  } else {
    Serial.println("IR Proximity Sensor OFF");
    return true;
  }
}

void fauxmoSetup() {
  // By default, fauxmoESP creates it's own webserver on the defined port
  // The TCP port must be 80 for gen3 devices (default is 1901)
  // This has to be done before the call to enable()
  fauxmo.createServer(true);  // not needed, this is the default value
  fauxmo.setPort(80);         // This is required for gen3 devices

  // You have to call enable(true) once you have a WiFi connection
  // You can enable or disable the library at any moment
  // Disabling it will prevent the devices from being discovered and switched
  fauxmo.enable(true);

  // You can use different ways to invoke alexa to modify the devices state:
  // "Alexa, turn yellow lamp on"
  // "Alexa, turn on yellow lamp
  // "Alexa, set yellow lamp to fifty" (50 means 50% of brightness, note, this example does not use this functionality)

  // Add virtual devices
  fauxmo.addDevice(CURTAIN);

  fauxmo.onSetState([](unsigned char device_id, const char* device_name, bool state, unsigned char value) {
    // Callback when a command from Alexa is received.
    // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
    // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
    // Just remember not to delay too much here, this is a callback, exit as soon as possible.
    // If you have to do something more involved here set a flag and process it in your main loop.

    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

    // Checking for device_id is simpler if you are certain about the order they are loaded and it does not change.
    // Otherwise comparing the device_name is safer.

    if (strcmp(device_name, CURTAIN) == 0) {
      if (state == HIGH) {
        takeActionToSwitchOnCurtain = true;
      } else {
        takeActionToSwitchOffCurtain = true;
      }
      curtainValue = (int)value;
    }
  });
}

void fauxmoLoop() {
  // fauxmoESP uses an async TCP server but a sync UDP server
  // Therefore, we have to manually poll for UDP packets
  fauxmo.handle();

  // This is a sample code to output free heap every 5 seconds
  // This is a cheap way to detect memory leaks
  static unsigned long last = millis();
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
  }
}

void checkForChangeAndSendToAlexa() {
  // There was a change in the curtain status. Lets send it across to cloud and alexa
  if (prevLoop_isCurtainOpen != isCurtainOpen()) {
    // If your device state is changed by any other means (MQTT, physical button,...)
    // you can instruct the library to report the new state to Alexa on next request:
    //fauxmo.setState(CURTAIN, isCurtainOpen ? true : false, curtainValue);
    sendSensorValueToAlexa(CURTAINKEY, isCurtainOpen() ? "open" : "closed");
  }
  prevLoop_isCurtainOpen = isCurtainOpen();
}

void checkForErrorAndSendToAlexa() {
  if (expected_isCurtainOpen != isCurtainOpen()) {
    // If your device state is changed by any other means (MQTT, physical button,...)
    // you can instruct the library to report the new state to Alexa on next request:
    //fauxmo.setState(CURTAIN, isCurtainOpen ? true : false, curtainValue);
    sendSensorValueToAlexa(CURTAINKEY, "error");
    expected_isCurtainOpen = isCurtainOpen();
  }
}

void checkAndTakeAction() {
  if (takeActionToSwitchOnCurtain) {
    expected_isCurtainOpen = true;
    openCurtain();
    takeActionToSwitchOnCurtain = false;
  }

  if (takeActionToSwitchOffCurtain) {
    expected_isCurtainOpen = false;
    closeCurtain();
    takeActionToSwitchOffCurtain = false;
  }
}