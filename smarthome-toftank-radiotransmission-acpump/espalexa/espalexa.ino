#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

#define WIFI_SSID "GTS"
#define WIFI_PASS "0607252609"
const String DROID_ID = "C3PO";

void setup() {
  Serial.begin(9600);  // Set the baud rate to 9600 bps
  wifiSetup();
}

void loop() {
  // Check if data is available to read from ATmega328
  if (Serial.available() > 0) {
    // Read the string data from ATmega328
    String receivedData = Serial.readStringUntil('\n');
    //Serial.println(receivedData);
    // Check if the received message starts with "WaterTank"
    if (receivedData.startsWith("WaterTank ")) {
      // Extract the substring after "WaterTank"
      String extractedData = receivedData.substring(10);  // 9 is the length of "WaterTank"
      sendSensorValueToAlexa("WaterTank", extractedData);
      //Serial.println("Sent to alexa");
    }
  }
}

void wifiSetup() {
  WiFi.mode(WIFI_STA);
  //Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print(".");
    delay(100);
  }
  //Serial.println();
  // Connected!
  //Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void sendSensorValueToAlexa(String name, String reading) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String fullUrl = "https://home-automation.vadrin.com/droid/" + DROID_ID + "/upsert/intent/" + name + "/reading/" + reading;
  //Serial.println("Requesting " + fullUrl);
  if (https.begin(client, fullUrl)) {
    int httpCode = https.GET();
    //Serial.println("============== Response code: " + String(httpCode));
    if (httpCode > 0) {
      //Serial.println(https.getString());
    }
    https.end();
  } else {
    //Serial.printf("[HTTPS] Unable to connect\n");
  }
}