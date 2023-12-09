#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BearSSLHelpers.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <TimeLib.h>
#include "stringutils.h"
#include "fauxmoESP.h"
#include <WiFiClientSecure.h>

// Configurations
#define DISPLAY_TYPE LCD_BIG_DISPLAY  // LCD_DISPLAY, MATRIX_DISPLAY, LCD_BIG_DISPLAY
#define BME_TYPE NO_BME               // BME680, BME280, NO_BME
#define ALARM_TYPE EXTERNAL_ALARM     // INTERNAL_ALARM, EXTERNAL_ALARM
const char* ssid = "XXX";
const char* password = "YYY";
const String droid = "ZZZ";
const String DISPLAY_HEADER = "DROID HOME";               //"\x03 DROID \x03" for Matrix, "DROID HOME" for LCD
#define DEVICE "DESK ALARM"                               //"DESK ALARM", "HALL ALARM"
String DEVICEKEY = "DeskAlarm";                           //"DeskAlarm", "HallAlarm"
const unsigned long PAYLOAD_SAMPLING_FREQUENCY = 120000;  //ms, 60000 for LCD, 120000 for Matrix, 120000 for LCD_BIG
const unsigned long SCREEN_CYCLE_FREQUENCY = 15500;       //ms, 5000 for LCD, 15500 for Matrix, 15500 for LCD_BIG
const int PIR_PIN = 14;                                   //14 for LCD, 2 for Matrix
const int ALARM_PIN = 30;                                 //30 for LCD, 3 for Matrix
const unsigned long PIR_TURN_OFF_TIME = 300000;           //ms, 300000 for LCD, 120000 for Matrix
float PRECISSION_TEMP = 1.0;                              //degrees
float PRECISSION_HUMID = 2.0;                             //percentage
float PRECISSION_AQI = 10.0;                              //value
const long DIM_DURATION = 500;                            // interval for high state (milliseconds) 500 for LCD, 2000 for Matrix
const long NOT_DIM_DURATION = 2000;                       // interval for low state (milliseconds) 2000 for LCD, 500 for Matrix
#define SEALEVELPRESSURE_HPA (1013.25)
const unsigned long ALEXA_LAG = 10000;
const unsigned long API_TIMEOUT = 3000;

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
boolean motionDetectedRecently = true;
//BME readings
float bme_readTemperature, bme_readPressure, bme_readHumidity, bme_readAltitude, bme_aqi, bme_aqiAccuracy;
float prev_bme_readTemperature, prev_bme_readHumidity, prev_bme_aqi;
boolean BMEChangeDetected = true;
SensorData getSpecificSensorData(String keyToGet);  // Helper functions declarations
int alarmHrs = 0;                                   //24 hrs format
int alarmMins = 0;                                  //0 to 60
boolean alarmEnabled;
boolean isAlarmState;
boolean isAttentionState;
fauxmoESP fauxmo;
volatile boolean takeActionToSwitchOnDevice, takeActionToSwitchOffDevice;
volatile int deviceValue = 255;
String lastSentMessage;
unsigned long attention_previousMillis = 0;  // will store last time LED was updated
bool attention_isHigh = false;               // flag to track the state
static unsigned long last = millis();
unsigned long sendToAlexaTime;
boolean alarmChangeDetectedRecently;

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
#define NO_BME 3
#ifdef BME_TYPE
#if BME_TYPE == BME280
#include "bme280.h"       // Include and use the bme280 library
#elif BME_TYPE == BME680  // Include and use the BME680 library
#include "bme680.h"
#elif BME_TYPE == NO_BME  // Dont include anything
#include "nobme.h"
#else
#error "Invalid library selection."
#endif
#else
#error "Library selection not defined."
#endif

#define INTERNAL_ALARM 1
#define EXTERNAL_ALARM 2
#ifdef ALARM_TYPE
#if ALARM_TYPE == INTERNAL_ALARM
#include "internalalarm.h"          // Include and use the internalalarm library
#elif ALARM_TYPE == EXTERNAL_ALARM  // Include and use the externalalarm library
#include "externalalarm.h"
#else
#error "Invalid library selection."
#endif
#else
#error "Library selection not defined."
#endif

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);       //Setup the PIR
  pinMode(ALARM_PIN, OUTPUT);    //Setup the ALARM
  digitalWrite(ALARM_PIN, LOW);  //Set it to low when starting
  setupDisplay();
  setupWifi();
  setupBME();
  fauxmoSetup();
  //Lets fetch and parse once to be ready to display immediatly.
  while (timeStatus() != timeSet) {
    Serial.println("trying to fetch time");
    fetchAndLoadCurrentTimeFromWeb();
  }
  fetchPayload();
  parsePayload();
  setAlarmTimeFromCloudToDevice();
}

void loop() {
  //BME Section
  loadBMEReadings();
  checkAndsendToAlexaBMEReadings();

  //Alarm Section
  fauxmoLoop();
  checkAndSetAlarm();
  checkAndStartAlarm();
  checkAndsendToAlexaAlarmReadings();

  //Display Section
  loadMotionReadings();
  if (motionDetectedRecently) {
    fetchAndParseFromURLFrequently();
    preProcessAndSetDisplayFrequently();
  } else {
    turnOffDisplay();  //switch off everything by Clearing the display and turn off the backlight
  }

  // Loops
  alarmLoop();
  displayLoop((isAttentionState || isAlarmState) && attention_dim());
}

void preProcessAndSetDisplayFrequently() {
  unsigned long currentTime = millis();
  if (payload != "" && !globalDataEntries.empty() && globalDataEntries.size() > 0 && (currentTime - lastScreenChangeTime > SCREEN_CYCLE_FREQUENCY)) {
    indexToDisplay++;
    if (indexToDisplay >= globalDataEntries.size()) {
      indexToDisplay = 0;
    }
    String preProcess = preProcessMessage(String(globalDataEntries[indexToDisplay].key) + String(" is ") + String(globalDataEntries[indexToDisplay].deviceReading));
    setDisplayMessage(preProcess);
    isAttentionState = globalDataEntries[indexToDisplay].inNeedOfAttention;
    lastScreenChangeTime = currentTime;
  } else {
    if (alarmChangeDetectedRecently) {
      String alarmHrMinStr = DEVICEKEY + String(": ") + formatHrsMins(alarmHrs, alarmMins, true);
      alarmHrMinStr = camelCaseToWordsUntillFirstColon(alarmHrMinStr);
      alarmHrMinStr = convertToUppercaseBeforeColon(alarmHrMinStr);
      setDisplayMessage(alarmHrMinStr);
      alarmChangeDetectedRecently = false;
    }
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
  Serial.println("begining the data fetch now");
  // Create a WiFiClientSecure object for HTTPS
  BearSSL::WiFiClientSecure client;
  client.setInsecure();            // Ignore SSL certificate validation (use for testing only)
  client.setTimeout(API_TIMEOUT);  // 1 second timeout

  // Make a GET request
  HTTPClient http;
  http.setTimeout(API_TIMEOUT);  // 1 second timeout

  // Set the WiFiClientSecure object for the HTTPClient
  http.begin(client, serverAddress + endpoint);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response Code: ");
    Serial.println(httpResponseCode);
    if (httpResponseCode == HTTP_CODE_OK) {
      payload = http.getString();
      Serial.println("Response payload:");
      Serial.println(payload);
    } else {
      Serial.print("Error on HTTPS request2: ");
      Serial.println(httpResponseCode);
    }
  } else {
    Serial.print("Error on HTTPS request3: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void parsePayload() {
  Serial.println("begining the payload parse now");
  DynamicJsonDocument doc(2048);  // Adjust the size based on your JSON data size

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
      sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%20degree%20celsius%20at%20" + String((int)bme_readHumidity) + "%25%20humidity");
    if (bme_readTemperature != 0 && BME_TYPE == 2) {
      sendSensorValueToAlexa("Indoor", String((int)bme_readTemperature) + "%20degree%20celsius%20at%20" + String((int)bme_readHumidity) + "%25%20humidity%2C%20" + String((int)bme_aqi) + "%20" + capitalizeFirstNCharacters("aqi", bme_aqiAccuracy));
    }
    BMEChangeDetected = false;
  }
}

boolean sendSensorValueToAlexa(String name, String reading) {
  boolean toReturn = false;
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(API_TIMEOUT);
  HTTPClient https;
  https.setTimeout(API_TIMEOUT);
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

void fetchAndLoadCurrentTimeFromWeb() {
  HTTPClient http;
  WiFiClient wifiClient;
  wifiClient.setTimeout(API_TIMEOUT);  // 1 second timeout
  http.setTimeout(API_TIMEOUT);        // 1 second timeout

  if (http.begin(wifiClient, WORLD_TIME_API)) {
    int httpCode = http.GET();

    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String webPayload = http.getString();

        // Parse the JSON response to extract the time and date
        int location = webPayload.indexOf("datetime");
        if (location != -1) {
          String time = webPayload.substring(location + 11, location + 30);  // Extract the time part from the response
          Serial.println(time);
          int year = time.substring(0, 4).toInt();
          int month = time.substring(5, 7).toInt();
          int day = time.substring(8, 10).toInt();
          int hour = time.substring(11, 13).toInt();
          int minute = time.substring(14, 16).toInt();
          int second = time.substring(17, 19).toInt();
          /*
          Serial.println(year);
          Serial.println(month);
          Serial.println(day);
          Serial.println(hour);
          Serial.println(minute);
          Serial.println(second);
          */
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

void checkAndStartAlarm() {
  if (alarmEnabled && hour() == alarmHrs && minute() == alarmMins) {
    if (!isAlarmState) {
      startAlarm();
      isAlarmState = true;
    }
  } else {
    if (isAlarmState) {
      stopAlarm();
      isAlarmState = false;
    }
  }
}

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

void checkAndsendToAlexaAlarmReadings() {
  if (millis() > sendToAlexaTime) {
    String meessageToSend;
    if (alarmEnabled) {
      meessageToSend = formatHrsMins(alarmHrs, alarmMins, true);
      meessageToSend = replaceFirstOccurrence(meessageToSend, ":", "%3A");
      meessageToSend = replaceFirstOccurrence(meessageToSend, " ", "%20");
    } else
      meessageToSend = "off";
    if (!areStringsEqual(meessageToSend, lastSentMessage)) {
      // If your device state is changed by any other means (MQTT, physical button,...)
      // you can instruct the library to report the new state to Alexa on next request:
      fauxmo.setState(DEVICE, alarmEnabled ? true : false, deviceValue);
      if (sendSensorValueToAlexa(DEVICEKEY, meessageToSend))
        lastSentMessage = meessageToSend;
    }
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

void checkAndSetAlarm() {
  if (takeActionToSwitchOnDevice) {
    alarmEnabled = true;
    takeActionToSwitchOnDevice = false;
    sendToAlexaTime = millis() + ALEXA_LAG;
    alarmChangeDetectedRecently = true;
    populateHrsMinsFromDeviceValue();
  }

  if (takeActionToSwitchOffDevice) {
    alarmEnabled = false;
    takeActionToSwitchOffDevice = false;
    sendToAlexaTime = millis() + ALEXA_LAG;
    populateHrsMinsFromDeviceValue();
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
  if (millis() - last > 10000) {
    last = millis();
    Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
  }
}

void setAlarmTimeFromCloudToDevice() {
  String timeString = getSpecificSensorData(DEVICEKEY).deviceReading;

  if (timeString.isEmpty())
    return;

  // Convert String to const char*
  const char* timeStringCstr = timeString.c_str();

  // Parse the time string
  int hours, minutes;
  char ampm[3];

  // Check the return value of sscanf to ensure successful parsing
  if (sscanf(timeStringCstr, "%d:%d %2s", &hours, &minutes, ampm) == 3) {
    // Adjust hours based on AM/PM
    if (strcmp(ampm, "PM") == 0 && hours != 12) {
      hours += 12;
    } else if (strcmp(ampm, "AM") == 0 && hours == 12) {
      hours = 0;
    }

    // Update global variables
    alarmEnabled = true;
    alarmHrs = hours;
    alarmMins = minutes;
    reversePopulateDeviceValueFromHrsMins();
    /*
    Serial.println(alarmEnabled);
    Serial.println(alarmHrs);
    Serial.println(alarmMins);
    Serial.println(deviceValue);
    */
    // Update alexa also
    fauxmo.setState(DEVICE, alarmEnabled ? true : false, deviceValue);
    Serial.println("Alarm set from cloud");
  } else {
    // Handle the case where parsing fails
    Serial.println("Error: Invalid time format");
    // You can add additional error-handling logic here if needed
  }
}

void reversePopulateDeviceValueFromHrsMins() {
  // Ensure that the values are within the valid range
  float m_alarmHrs = alarmHrs % 24;
  float m_alarmMins = alarmMins % 60;

  // Calculate total minutes since midnight
  float minutesSinceMidnight = (m_alarmHrs * 60) + m_alarmMins;

  // Calculate the percentage based on 15 minutes per percent
  float perc = minutesSinceMidnight / 15.0;

  // Calculate the device value based on the percentage and the range [0, 255]
  deviceValue = (perc / 100.0) * 255;
}