#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h> //you can download the library from https://github.com/gv-prashanth/arduino-projects/tree/master/libraries
#endif
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_INA219.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define WIFI_SSID "XXXXXX"
#define WIFI_PASS "YYYYYY"

Adafruit_BME280 bme;
Adafruit_INA219 ina219;
ADC_MODE(ADC_VCC);
const String DROID_ID = "C3PO";
const int TIME_BETWEEN_SAMPLING = 900000;//ms

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Wi-Fi connection
  wifiSetup();

  Serial.println(F("BME680 Initializing..."));
  Wire.begin(0, 2);  // SDA, SCL
  bme.begin(0x76);

  Serial.println(F("INA219 Initializing..."));
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  sendToAlexaBMEReadings();
  sendToAlexaINAReadings();
  delay(TIME_BETWEEN_SAMPLING);
}

void sendToAlexaINAReadings() {
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
  float espVoltage = ESP.getVcc() / 1000.00;
  
  Serial.print("ESP Voltage = ");
  Serial.print(espVoltage);
  Serial.println(" V");
  
  Serial.print("Bus Voltage = ");
  Serial.print(busvoltage);
  Serial.println(" V");

  Serial.print("Shunt Voltage = ");
  Serial.print(shuntvoltage);
  Serial.println(" mV");

  Serial.print("Load Voltage = ");
  Serial.print(loadvoltage);
  Serial.println(" V");

  Serial.print("Load Current = ");
  Serial.print(current_mA);
  Serial.println(" mA");

  Serial.print("Load power = ");
  Serial.print(power_mW);
  Serial.println(" mW");

  //sendSensorValueToAlexa("SolarPanel", "load%20voltage%20is%20"+String(loadvoltage)+"%20volts%2C%20load%20current%20is%20"+String(current_mA)+"%20milli%20amperes%2C%20load%20power%20is%20"+String(power_mW)+"%20milli%20watts%2E%20ESP%20voltage%20is%20"+String(espVoltage)+"%20volts");
  sendSensorValueToAlexa("SolarPanel", "generating%20"+String((int)(power_mW/1000))+"%20watts%20at%20"+String((int)loadvoltage)+"%20volts");
}

void sendToAlexaBMEReadings() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  
  //sendSensorValueToAlexa("Weather", String(bme.readTemperature())+"%20degree%20celsius%2C%20humidity%20is%20"+String(bme.readHumidity())+"%25%2C%20pressure%20is%20"+String(bme.readPressure() / 100.0F)+"%20hectopascal%2C%20altitude%20is%20"+String(bme.readAltitude(SEALEVELPRESSURE_HPA))+"%20meters");
  sendSensorValueToAlexa("Weather", String((int)bme.readTemperature())+"%20degree%20celsius%20at%20"+String((int)bme.readHumidity())+"%25%20humidity");
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
