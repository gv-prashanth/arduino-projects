#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

// -------------------------------DONT TOUCH------------------------------------
fauxmoESP fauxmo;
volatile boolean takeActionToSwitchOnDevice, takeActionToSwitchOffDevice;
volatile int deviceValue = 255;
String lastSentMessage;
// -----------------------------------------------------------------------------

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
  fauxmo.addDevice(DEVICE);

  fauxmo.onSetState([](unsigned char device_id, const char* device_name, bool state, unsigned char value) {
    // Callback when a command from Alexa is received.
    // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
    // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
    // Just remember not to delay too much here, this is a callback, exit as soon as possible.
    // If you have to do something more involved here set a flag and process it in your main loop.

    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

    // Checking for device_id is simpler if you are certain about the order they are loaded and it does not change.
    // Otherwise comparing the device_name is safer.

    if (strcmp(device_name, DEVICE) == 0) {
      if (state == HIGH) {
        takeActionToSwitchOnDevice = true;
      } else {
        takeActionToSwitchOffDevice = true;
      }
      deviceValue = (int)value;
    }
  });
}

void fauxmoLoop() {
  // fauxmoESP uses an async TCP server but a sync UDP server
  // Therefore, we have to manually poll for UDP packets
  fauxmo.handle();

  // This is a sample code to output free heap every 5 seconds
  // This is a cheap way to detect memory leaks
  //static unsigned long last = millis();
  //if (millis() - last > 5000) {
  //  last = millis();
  //  Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
  //}
}

void checkForChangeAndSendToAlexa() {
  String meessageToSend;
  if(alarmEnabled)
    meessageToSend = String(alarmHrs) + "Hr%20" + String(alarmMins)+ "Min";
  else
    meessageToSend = "off";
  if (!areStringsEqual(meessageToSend, lastSentMessage)) {
    // If your device state is changed by any other means (MQTT, physical button,...)
    // you can instruct the library to report the new state to Alexa on next request:
    fauxmo.setState(DEVICE, alarmEnabled ? true : false, deviceValue);
    if (sendSensorValueToAlexa(DEVICEKEY, meessageToSend))
      lastSentMessage = meessageToSend;
  }
}

void populateHrsMinsFromDeviceValue() {
  float perc = (float(deviceValue) / 255.0) * 100;
  int minutesSinceMidnight = perc * 15;  //since 1% aprox equals 15min
  // Calculate hours and minutes
  int hoursAlarm = minutesSinceMidnight / 60;
  int minutesAlarm = minutesSinceMidnight % 60;
  // Ensure that the values are within the valid range
  hoursAlarm = hoursAlarm % 24;
  minutesAlarm = minutesAlarm % 60;
  //set alarmhrs
  alarmHrs = hoursAlarm;
  alarmMins = minutesAlarm;
}

void checkAndTakeAction() {
  if (takeActionToSwitchOnDevice) {
    alarmEnabled = true;
    takeActionToSwitchOnDevice = false;
    populateHrsMinsFromDeviceValue();
  }

  if (takeActionToSwitchOffDevice) {
    alarmEnabled = false;
    takeActionToSwitchOffDevice = false;
    populateHrsMinsFromDeviceValue();
  }
}

void alexaSetup() {
  fauxmoSetup();
}

void alexaLoop() {
  fauxmoLoop();
  checkAndTakeAction();
  checkForChangeAndSendToAlexa();
}