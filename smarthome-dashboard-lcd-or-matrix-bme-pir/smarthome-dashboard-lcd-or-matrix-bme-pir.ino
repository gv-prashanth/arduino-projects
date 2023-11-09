#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BearSSLHelpers.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>

// Configurations
#define DISPLAY_TYPE MATRIX_DISPLAY  // LCD_DISPLAY, MATRIX_DISPLAY
#define BME_TYPE BME680              // BME680, BME280
const char* ssid = "XXX";
const char* password = "YYY";
const String droid = "ZZZ";
const unsigned long PAYLOAD_SAMPLING_FREQUENCY = 120000;  //ms, 60000 for LCD, 120000 for Matrix
const unsigned long SCREEN_CYCLE_FREQUENCY = 15500;       //ms, 5000 for LCD, 15500 for Matrix
const int PIR_PIN = 14;                                   // PIR sensor input pin 14. NOT ALL PINS WILL WORK.
const unsigned long PIR_TURN_OFF_TIME = 60000;            //ms
float PRECISSION_TEMP = 1.0;                              //degrees
float PRECISSION_HUMID = 2.0;                             //percentage
float PRECISSION_AQI = 10.0;                              //value
#define SEALEVELPRESSURE_HPA (1013.25)

// Dont touch below
const String serverAddress = "https://home-automation.vadrin.com";  // Note the "https://" prefix
const String endpoint = "/droid/" + droid + "/intents";
String payload;
int indexToDisplay = 0;
unsigned long lastFetchTime, lastScreenChangeTime, motionTimestamp;
struct SensorData {
  int jsonIndex;
  String key;
  String deviceReading;
  String readingTime;
};
std::vector<SensorData> globalDataEntries;  // Global vector to store the data
boolean motionDetectedRecently;
//BME readings
float bme_readTemperature, bme_readPressure, bme_readHumidity, bme_readAltitude, bme_aqi, bme_aqiAccuracy;
float prev_bme_readTemperature, prev_bme_readHumidity, prev_bme_aqi;
boolean BMEChangeDetected;
SensorData getSpecificSensorData(String keyToGet);                                // Helper functions declarations
String replaceString(String input, const String& search, const String& replace);  // Helper functions declarations

#define LCD_DISPLAY 1
#define MATRIX_DISPLAY 2
#ifdef DISPLAY_TYPE
#if DISPLAY_TYPE == LCD_DISPLAY
#include "lcddisplay.h"               // Include and use the LCD display library
#elif DISPLAY_TYPE == MATRIX_DISPLAY  // Include and use the matrix display library
#include "matrixdisplay.h"
#else
#error "Invalid library selection."
#endif
#else
#error "Library selection not defined."
#endif

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
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);  //Setup the PIR
  setupDisplay();
  setupWifi();
  setupBME();
  BMEChangeDetected = true;
  motionDetectedRecently = true;
  //Lets fetch and parse once to be ready to display immediatly.
  fetchPayload();
  parsePayload();
}

void loop() {
  unsigned long currentTime = millis();

  loadBMEReadings();
  checkAndsendToAlexaBMEReadings();

  if (digitalRead(PIR_PIN) == HIGH) {
    motionTimestamp = currentTime;
    motionDetectedRecently = true;
  } else {
    if (motionDetectedRecently) {
      if (currentTime - motionTimestamp > PIR_TURN_OFF_TIME) {
        Serial.println("Motion ended.");
        motionDetectedRecently = false;
      }
    }
  }

  if (motionDetectedRecently) {
    if (currentTime - lastFetchTime > PAYLOAD_SAMPLING_FREQUENCY) {
      fetchPayload();
      parsePayload();
      lastFetchTime = currentTime;
    }
    if (payload != "" && (currentTime - lastScreenChangeTime > SCREEN_CYCLE_FREQUENCY)) {
      indexToDisplay++;
      if (indexToDisplay >= globalDataEntries.size()) {
        indexToDisplay = 0;
      }
      String preProcess = preProcessMessage(String(globalDataEntries[indexToDisplay].key) + String(" is ") + String(globalDataEntries[indexToDisplay].deviceReading));
      setDisplayMessage(preProcess);
      lastScreenChangeTime = currentTime;
    }
  } else {
    //switch off everything by Clearing the display and turn off the backlight
    turnOffDisplay();
  }

  displayScreen();
}

void setupWifi() {
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void fetchPayload() {
  // Create a WiFiClientSecure object for HTTPS
  BearSSL::WiFiClientSecure client;
  client.setInsecure();  // Ignore SSL certificate validation (use for testing only)

  // Make a GET request
  HTTPClient http;

  // Set the WiFiClientSecure object for the HTTPClient
  http.begin(client, serverAddress + endpoint);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response Code: ");
    Serial.println(httpResponseCode);

    payload = http.getString();
    Serial.println("Response payload:");
    Serial.println(payload);

  } else {
    Serial.print("Error on HTTPS request: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void parsePayload() {
  DynamicJsonDocument doc(3072);  // Adjust the size based on your JSON data size

  // Deserialize the JSON data
  DeserializationError error = deserializeJson(doc, payload);

  if (!error) {
    int jsonIndex = 0;
    globalDataEntries.clear();
    for (JsonPair kv : doc.as<JsonObject>()) {
      SensorData entry;
      entry.jsonIndex = jsonIndex;
      entry.key = String(kv.key().c_str());
      entry.deviceReading = String(kv.value()["deviceReading"]);
      entry.readingTime = String(kv.value()["readingTime"]);
      globalDataEntries.push_back(entry);
      jsonIndex++;
    }
  }
}

String preProcessMessage(String str) {
  str = replaceString(str, " is at ", ": ");
  str = replaceString(str, " is ", ": ");
  str = convertToUppercaseBeforeColon(str);
  str = replaceMultipleSpaces(str);
  str = modifyStringToCapitalAfterColon(str);
  str = trimString(str);
  str = removeLastFullStop(str);
  str = replaceString(str, " degree celsius", String((char)223) + "C");
  return str;
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
      sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%20degree%20celsius%20" + String((int)bme_readHumidity) + "%25%20humidity");  //sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%C2%B0C%2C%20" + String((int)bme_readHumidity) + "%25%20humidity");
    if (bme_readTemperature != 0 && BME_TYPE == 2) {
      if (bme_aqiAccuracy > 0)
        sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%20degree%20celsius%20" + String((int)bme_readHumidity) + "%25%20humidity%2C%20" + String((int)bme_aqi) + "%20AQI");  //sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%C2%B0C%2C%20" + String((int)bme_readHumidity) + "%25%20humidity%2C%20" + String((int)bme_aqi) + "%20AQI");
      else
        sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%20degree%20celsius%20" + String((int)bme_readHumidity) + "%25%20humidity%2C%20" + String((int)bme_aqi) + "%20aqi");  //sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%C2%B0C%2C%20" + String((int)bme_readHumidity) + "%25%20humidity%2C%20" + String((int)bme_aqi) + "%20aqi");
    }
    BMEChangeDetected = false;
  }
}

void sendSensorValueToAlexa(String name, String reading) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String fullUrl = serverAddress + "/droid/" + droid + "/upsert/intent/" + name + "/reading/" + reading;
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

String replaceString(String input, const String& search, const String& replace) {
  int index = 0;
  while ((index = input.indexOf(search, index)) != -1) {
    input = input.substring(0, index) + replace + input.substring(index + search.length());
    index += replace.length();
  }
  return input;
}

String convertToUppercaseBeforeColon(String input) {
  int colonIndex = input.indexOf(':');  // Find the position of the first colon

  if (colonIndex != -1) {
    // Extract the part before the colon
    String partBeforeColon = input.substring(0, colonIndex);

    // Convert the extracted part to uppercase
    partBeforeColon.toUpperCase();

    // Construct the final output string
    String output = partBeforeColon + input.substring(colonIndex);

    return output;
  } else {
    // If no colon is found, return the input string as it is
    return input;
  }
}

String replaceMultipleSpaces(String input) {
  while (input.indexOf("  ") != -1) {
    input.replace("  ", " ");
  }
  return input;
}

String modifyStringToCapitalAfterColon(String input) {
  // Find the position of ": "
  int colonSpaceIndex = input.indexOf(": ");

  // Check if ": " was found
  if (colonSpaceIndex != -1 && colonSpaceIndex < input.length() - 2) {
    // Get the character after ": "
    char charToCapitalize = input.charAt(colonSpaceIndex + 2);

    // Check if the character is an alphabet letter
    if (isAlpha(charToCapitalize)) {
      // Convert the character to uppercase
      charToCapitalize = toupper(charToCapitalize);

      // Replace the original character with the uppercase one
      input.setCharAt(colonSpaceIndex + 2, charToCapitalize);
    }
  }

  // Return the modified string
  return input;
}

String removeLastFullStop(String inputString) {
  int stringLength = inputString.length();

  // Check if the string is empty or if the last character is not a period
  if (stringLength == 0 || inputString.charAt(stringLength - 1) != '.') {
    return inputString;
  }

  // Remove the last character (period)
  inputString.remove(stringLength - 1);
  return inputString;
}

String trimString(String inputString) {
  // Trim leading spaces
  int startIndex = 0;
  while (inputString.charAt(startIndex) == ' ') {
    startIndex++;
  }

  // Trim trailing spaces
  int endIndex = inputString.length() - 1;
  while (inputString.charAt(endIndex) == ' ') {
    endIndex--;
  }

  // Extract the trimmed substring
  String trimmedString = inputString.substring(startIndex, endIndex + 1);

  return trimmedString;
}

SensorData getSpecificSensorData(String keyToGet) {
  SensorData result;
  result.key = keyToGet;  // Key to search for

  for (const SensorData& entry : globalDataEntries) {
    if (entry.key == result.key) {
      // Found a matching entry with the key "Clock"
      result = entry;
      break;  // Exit the loop once found
    }
  }

  return result;
}