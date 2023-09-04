#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h> //you can download the library from https://github.com/gv-prashanth/arduino-projects/tree/master/libraries
#endif
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_INA219.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define WIFI_SSID "XXXXXX"
#define WIFI_PASS "YYYYYY"

Adafruit_BME680 bme; // I2C
Adafruit_INA219 ina219;
ADC_MODE(ADC_VCC);
const String DROID_ID = "N5X9";
const int TIME_BETWEEN_SAMPLING = 120000;//ms

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Wi-Fi connection
  wifiSetup();

  Serial.println(F("BME680 Initializing..."));
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

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
  printESPReadings();
  delay(TIME_BETWEEN_SAMPLING);
}

void printESPReadings() {
  float espVoltage = ESP.getVcc() / 1000.00;
  Serial.print("ESP Voltage = ");
  Serial.print(espVoltage);
  Serial.println(" V");
}

void sendToAlexaINAReadings() {
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);

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

  sendSensorValueToAlexa("SolarPanel", "%20at%20"+String(busvoltage)+"%20Volts");
}

void sendToAlexaBMEReadings() {
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  
  sendSensorValueToAlexa("Weather", "%20at%20"+String(bme.temperature)+"%20Centigrade%20and%20"+String()+"%25%20humidity");
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
