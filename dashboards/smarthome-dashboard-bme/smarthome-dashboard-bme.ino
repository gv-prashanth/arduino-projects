#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

// Configurations
#define BME_TYPE BME680               // BME680, BME280
const char* ssid = "XXXX";
const char* password = "YYYY";
const String DROID_ID_SEND = "ABCD";                      // Ideally should be same as DROID_ID_FETCH
String WEATHERKEY = "Outdoor Weather";
float PRECISSION_TEMP = 0.3;                              //degrees
float PRECISSION_HUMID = 5.0;                             //percentage
float PRECISSION_AQI = 10.0;                              //value
const unsigned long API_TIMEOUT = 3000;

// Dont touch below
const String serverAddress = "https://home-automation.vadrin.com";  // Note the "https://" prefix
//BME readings
float bme_readTemperature, bme_readPressure, bme_readHumidity, bme_readAltitude, bme_aqi, bme_aqiAccuracy;
float prev_bme_readTemperature, prev_bme_readHumidity, prev_bme_aqi, prev_bme_aqiAccuracy;
boolean BMEChangeDetected = true;
//BME types
#define BME280 1
#define BME680 2
#ifdef BME_TYPE
#if BME_TYPE == BME280
#include "bme280.h"       // Include and use the bme280 library
#elif BME_TYPE == BME680  // Include and use the BME680 library
#include "bme680.h"
#else
#error "Invalid library selection."
#endif
#else
#error "Library selection not defined."
#endif

void setup() {
  Serial.begin(115200);        //WARNING: IF YOU TURN SERIAL PRINT, THEN DISCONNECT SPEAKER. LIKEWISE IF YOU CONNECT SPEAKER, THEN DISABLE SERIAL PRINT
  setupWifi();
  setupBME();
}

void loop() {
  //BME Section
  loadBMEReadings();
  checkAndsendToAlexaBMEReadings();
}

void setupWifi() {
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void checkAndsendToAlexaBMEReadings() {
  if (BMEChangeDetected) {
    Serial.print("Temperature = ");
    Serial.print(bme_readTemperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme_readPressure);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme_readHumidity);
    Serial.println(" %");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme_readAltitude);
    Serial.println(" m");
    if (bme_readTemperature != 0 && BME_TYPE == 1)
      sendSensorValueToAlexa(WEATHERKEY, String(bme_readTemperature, 1) + "%C2%B0C%2C%20" + String((int)bme_readHumidity) + "%25RH");
    if (bme_readTemperature != 0 && BME_TYPE == 2) {
      sendSensorValueToAlexa(WEATHERKEY, String(bme_readTemperature, 1) + "%C2%B0C%2C%20" + String((int)bme_readHumidity) + "%25RH%2C%20" + String((int)bme_aqi) + capitalizeFirstNCharacters("aqi", bme_aqiAccuracy));
    }
    BMEChangeDetected = false;
  }
}

boolean sendSensorValueToAlexa(String name, String reading) {
  boolean toReturn = false;
  String encodedName = "";
  for (int i = 0; i < name.length(); i++) encodedName += (name.charAt(i) == ' ') ? "%20" : String(name.charAt(i));
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(API_TIMEOUT);
  HTTPClient https;
  https.setTimeout(API_TIMEOUT);
  String fullUrl = serverAddress + "/droid/" + DROID_ID_SEND + "/upsert/intent/" + encodedName + "/reading/" + reading;
  Serial.println("Requesting " + fullUrl);
  if (https.begin(client, fullUrl)) {
    int httpCode = https.GET();
    Serial.println("============== Response code: " + String(httpCode));
    if (httpCode > 0) {
      Serial.println(https.getString());
      toReturn = true;
    }
    https.end();
  } else {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
  return toReturn;
}

String capitalizeFirstNCharacters(String result, int n) {
  if (n >= 0 && n <= result.length()) {
    // Capitalize the first n characters of the string
    for (int i = 0; i < n; i++) {
      result[i] = toupper(result[i]);
    }
  } else {
    // Handle invalid input by returning the original string
    Serial.println("Invalid value of n. Returning the input string unchanged.");
  }
  return result;  // Return the capitalized string or the original string if input is invalid
}