#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BearSSLHelpers.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <TimeLib.h>
#include "audio.h"
#include "alarmaudio.h"
#include "welcomeaudio.h"

// Configurations
#define DISPLAY_TYPE LCD_BIG_DISPLAY  // LCD_DISPLAY, MATRIX_DISPLAY, LCD_BIG_DISPLAY
#define BME_TYPE BME280               // BME680, BME280
const char* ssid = "XXX";
const char* password = "YYY";
const String droid = "ZZZ";
const String DISPLAY_HEADER = "DROID HOME";               //"\x03 DROID \x03" for Matrix, "DROID HOME" for LCD
#define DEVICE "DESK CLOCK"                               //"DESK CLOCK"
String DEVICEKEY = "DeskClock";                           //"DeskClock"
const unsigned long PAYLOAD_SAMPLING_FREQUENCY = 120000;  //ms, 60000 for LCD, 120000 for Matrix, 120000 for LCD_BIG
const unsigned long SCREEN_CYCLE_FREQUENCY = 15500;       //ms, 5000 for LCD, 15500 for Matrix, 15500 for LCD_BIG
const int PIR_PIN = 14;                                   //14 for LCD, 2 for Matrix
const unsigned long PIR_TURN_OFF_TIME = 300000;           //ms
float PRECISSION_TEMP = 1.0;                              //degrees
float PRECISSION_HUMID = 2.0;                             //percentage
float PRECISSION_AQI = 10.0;                              //value
const long DIM_DURATION = 500;                            // interval for high state (milliseconds)
const long NOT_DIM_DURATION = 2000;                       // interval for low state (milliseconds)
#define SEALEVELPRESSURE_HPA (1013.25)

// Dont touch below
const String serverAddress = "https://home-automation.vadrin.com";  // Note the "https://" prefix
const String WORLD_TIME_API = "http://worldtimeapi.org/api/ip";     // Fetch the time from World Time API
const String endpoint = "/droid/" + droid + "/intents";
String payload;
int indexToDisplay = 0;
unsigned long lastFetchTime, lastScreenChangeTime, motionTimestamp;
struct SensorData {
  int jsonIndex;
  String key;
  String deviceReading;
  String readingTime;
  boolean inNeedOfAttention;
};
std::vector<SensorData> globalDataEntries;  // Global vector to store the data
boolean motionDetectedRecently;
//BME readings
float bme_readTemperature, bme_readPressure, bme_readHumidity, bme_readAltitude, bme_aqi, bme_aqiAccuracy;
float prev_bme_readTemperature, prev_bme_readHumidity, prev_bme_aqi;
boolean BMEChangeDetected;
SensorData getSpecificSensorData(String keyToGet);  // Helper functions declarations
//String replaceString(String input, const String& search, const String& replace);  // Helper functions declarations
String replaceFirstOccurrence(String input, const String& search, const String& replace);  // Helper functions declarations
int alarmHrs = 0;                                                                          //24 hrs format
int alarmMins = 0;                                                                         //0 to 60
boolean alarmEnabled;
boolean sendSensorValueToAlexa(String name, String reading);    // Helper functions declarations
boolean areStringsEqual(const String str1, const String str2);  // Helper functions declarations
#include "alexa.h"
boolean isAttention;
unsigned long startMillis;
boolean isWelcomePlaying;

#define LCD_DISPLAY 1
#define MATRIX_DISPLAY 2
#define LCD_BIG_DISPLAY 3
#ifdef DISPLAY_TYPE
#if DISPLAY_TYPE == LCD_DISPLAY
#include "lcddisplay.h"               // Include and use the LCD display library
#elif DISPLAY_TYPE == MATRIX_DISPLAY  // Include and use the matrix display library
#include "matrixdisplay.h"
#elif DISPLAY_TYPE == LCD_BIG_DISPLAY  // Include and use the LCD big display library
#include "lcdbigdisplay.h"
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
  alexaSetup();
  BMEChangeDetected = true;
  motionDetectedRecently = true;
  //Lets fetch and parse once to be ready to display immediatly.
  while (timeStatus() != timeSet) {
    Serial.println("trying to fetch time");
    fetchAndLoadCurrentTimeFromWeb();
  }
  fetchPayload();
  parsePayload();
  startMillis = millis();
  isWelcomePlaying = true;
}

void loop() {
  if (isWelcomePlaying && (millis() < (startMillis + 7000))) {
    playAudio(welcomeAudio, sizeof(welcomeAudio));
  } else {
    stopAudio();
    isWelcomePlaying = false;
  }

  loadBMEReadings();
  checkAndsendToAlexaBMEReadings();
  loadMotionReadings();
  if (motionDetectedRecently) {
    fetchAndParseFromURLFrequently();
    preProcessAndSetDisplayFrequently();
  } else {
    turnOffDisplay();  //switch off everything by Clearing the display and turn off the backlight
  }
  alexaLoop();
  boolean isAlarmPlaying = checkAndPlayAlarm();
  displayScreen((isAttention && attention_dim()) || (isAlarmPlaying && attention_dim()));
}

void preProcessAndSetDisplayFrequently() {
  unsigned long currentTime = millis();
  if (payload != "" && (currentTime - lastScreenChangeTime > SCREEN_CYCLE_FREQUENCY)) {
    indexToDisplay++;
    if (indexToDisplay >= globalDataEntries.size()) {
      indexToDisplay = 0;
    }
    String preProcess = preProcessMessage(String(globalDataEntries[indexToDisplay].key) + String(" is ") + String(globalDataEntries[indexToDisplay].deviceReading));
    setDisplayMessage(preProcess);
    isAttention = globalDataEntries[indexToDisplay].inNeedOfAttention;
    lastScreenChangeTime = currentTime;
  }
}

void fetchAndParseFromURLFrequently() {
  unsigned long currentTime = millis();
  if (currentTime - lastFetchTime > PAYLOAD_SAMPLING_FREQUENCY) {
    fetchPayload();
    parsePayload();
    lastFetchTime = currentTime;
  }
}

void loadMotionReadings() {
  unsigned long currentTime = millis();
  if (digitalRead(PIR_PIN) == HIGH) {
    motionTimestamp = currentTime;
    motionDetectedRecently = true;
    //Serial.println("motion is there.");
  } else {
    //Serial.println("motion is missing.");
    if (motionDetectedRecently) {
      if (currentTime - motionTimestamp > PIR_TURN_OFF_TIME) {
        Serial.println("Motion ended.");
        motionDetectedRecently = false;
      }
    }
  }
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

    //addWelcomeHeader
    SensorData welcome;
    welcome.jsonIndex = jsonIndex;
    welcome.key = String("Welcome");
    welcome.deviceReading = String(DISPLAY_HEADER);
    welcome.readingTime = String("now");
    welcome.inNeedOfAttention = false;
    globalDataEntries.push_back(welcome);
    jsonIndex++;

    //addCalendarToResponse
    SensorData calendar;
    calendar.jsonIndex = jsonIndex;
    calendar.key = String("Calendar");
    calendar.deviceReading = String(dayShortStr(weekday())) + ", " + String(monthShortStr(month())) + " " + String(day()) + ", " + String(year());
    calendar.readingTime = String("now");
    calendar.inNeedOfAttention = false;
    globalDataEntries.push_back(calendar);
    jsonIndex++;

    //addResponseDevices
    for (JsonPair kv : doc.as<JsonObject>()) {
      SensorData entry;
      entry.jsonIndex = jsonIndex;
      entry.key = String(kv.key().c_str());
      entry.deviceReading = String(kv.value()["deviceReading"]);
      entry.readingTime = String(kv.value()["readingTime"]);
      entry.inNeedOfAttention = areStringsEqual(String(kv.value()["inNeedOfAttention"]), "true");
      globalDataEntries.push_back(entry);
      jsonIndex++;
    }
  }
}

String preProcessMessage(String str) {
  //str = replaceFirstOccurrence(str, " is at ", ": ");
  str = replaceFirstOccurrence(str, " is ", ": ");
  str = camelCaseToWordsUntillFirstColon(str);
  str = convertToUppercaseBeforeColon(str);
  str = replaceMultipleSpaces(str);
  str = modifyStringToCapitalAfterColon(str);
  str = trimString(str);
  str = removeLastFullStop(str);
  return str;
}

String replaceFirstOccurrence(const String input, const String& match, const String& replace) {
  String output = input;
  int startPos = output.indexOf(match);

  if (startPos != -1) {
    output = output.substring(0, startPos) + replace + output.substring(startPos + match.length());
  }
  return output;
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
      sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%20degree%20celsius%20at%20" + String((int)bme_readHumidity) + "%25%20humidity");  //sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%C2%B0C%2C%20" + String((int)bme_readHumidity) + "%25%20humidity");
    if (bme_readTemperature != 0 && BME_TYPE == 2) {
      sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%20degree%20celsius%20at%20" + String((int)bme_readHumidity) + "%25%20humidity%2C%20" + String((int)bme_aqi) + "%20" + capitalizeFirstNCharacters("aqi", bme_aqiAccuracy));  //sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%C2%B0C%2C%20" + String((int)bme_readHumidity) + "%25%20humidity%2C%20" + String((int)bme_aqi) + "%20aqi");
    }
    BMEChangeDetected = false;
  }
}

boolean sendSensorValueToAlexa(String name, String reading) {
  boolean toReturn = false;
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
      toReturn = true;
    }
    https.end();
  } else {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
  return toReturn;
}

/*
String replaceString(String input, const String& search, const String& replace) {
  int index = 0;
  while ((index = input.indexOf(search, index)) != -1) {
    input = input.substring(0, index) + replace + input.substring(index + search.length());
    index += replace.length();
  }
  return input;
}
*/

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
      result = entry;
      break;  // Exit the loop once found
    }
  }

  return result;
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

void fetchAndLoadCurrentTimeFromWeb() {
  HTTPClient http;
  WiFiClient wifiClient;
  if (http.begin(wifiClient, WORLD_TIME_API)) {
    int httpCode = http.GET();

    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();

        // Parse the JSON response to extract the time and date
        int location = payload.indexOf("datetime");
        if (location != -1) {
          String time = payload.substring(location + 11, location + 30);  // Extract the time part from the response
          Serial.println(time);
          int year = time.substring(0, 4).toInt();
          int month = time.substring(5, 7).toInt();
          int day = time.substring(8, 10).toInt();
          int hour = time.substring(11, 13).toInt();
          int minute = time.substring(14, 16).toInt();
          int second = time.substring(17, 19).toInt();
          Serial.println(year);
          Serial.println(month);
          Serial.println(day);
          Serial.println(hour);
          Serial.println(minute);
          Serial.println(second);


          // Set the fetched date and time to the internal clock
          setTime(hour, minute, second, day, month, year);  // Set time (HH, MM, SS, DD, MM, YYYY)
          Serial.println("Time fetched and set.");
        } else {
          Serial.println("Failed to connect to the time server3");
        }
      } else {
        Serial.println("Failed to connect to the time server2");
      }
    }
    http.end();
  } else {
    Serial.println("Failed to connect to the time server1");
  }
}

String camelCaseToWordsUntillFirstColon(String input) {
  String output = "";
  bool colonFound = false;

  for (int i = 0; i < input.length(); i++) {
    if (input[i] == ':') {
      colonFound = true;
    }

    if (!colonFound) {
      if (i > 0 && isUpperCase(input[i]) && !isUpperCase(input[i - 1])) {
        output += " ";  // Add a space before adding the uppercase letter
        output += input[i];
      } else {
        output += input[i];
      }
    } else {
      output += input[i];
    }
  }

  return output;
}

boolean checkAndPlayAlarm() {
  if (isWelcomePlaying)
    return false;
  if (alarmEnabled && hour() == alarmHrs && minute() == alarmMins) {
    playAudio(alarmAudio, sizeof(alarmAudio));
    return true;
  } else {
    stopAudio();
    return false;
  }
}

boolean areStringsEqual(const String str1, const String str2) {
  // Convert String objects to char arrays for strcmp
  char charArray1[str1.length() + 1];
  char charArray2[str2.length() + 1];
  str1.toCharArray(charArray1, sizeof(charArray1));
  str2.toCharArray(charArray2, sizeof(charArray2));

  // Compare strings
  return strcmp(charArray1, charArray2) == 0;
}

unsigned long attention_previousMillis = 0;  // will store last time LED was updated
bool attention_isHigh = false;               // flag to track the state

// Function to return true every 2 seconds and false every 2 seconds
bool attention_dim() {
  // Get the current time
  unsigned long currentMillis = millis();

  // Check if it's time to toggle the state
  if (attention_isHigh && (currentMillis - attention_previousMillis >= DIM_DURATION)) {
    // Save the current time for the next iteration
    attention_previousMillis = currentMillis;

    // Switch to low state
    attention_isHigh = false;

    // Return the current state
    return false;
  } else if (!attention_isHigh && (currentMillis - attention_previousMillis >= NOT_DIM_DURATION)) {
    // Save the current time for the next iteration
    attention_previousMillis = currentMillis;

    // Switch to high state
    attention_isHigh = true;

    // Return the current state
    return true;
  }

  // If it's not time to toggle, return the current state
  return attention_isHigh;
}