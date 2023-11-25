#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BearSSLHelpers.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <TimeLib.h>
#include "stringutils.h"
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SNoDAC.h"
#include "fauxmoESP.h"
#include <WiFiClientSecure.h>
#include "alarmaudio.h"
#include "welcomeaudio.h"

// Configurations
#define DISPLAY_TYPE LCD_BIG_DISPLAY  // LCD_DISPLAY, MATRIX_DISPLAY, LCD_BIG_DISPLAY
#define BME_TYPE BME280               // BME680, BME280
const char* ssid = "XXX";
const char* password = "YYY";
const String droid = "ZZZ";
const String DISPLAY_HEADER = "DROID HOME";               //"\x03 DROID \x03" for Matrix, "DROID HOME" for LCD
#define DEVICE "DESK ALARM"                               //"DESK ALARM", "HALL ALARM"
String DEVICEKEY = "DeskAlarm";                           //"DeskAlarm", "HallAlarm"
const unsigned long PAYLOAD_SAMPLING_FREQUENCY = 120000;  //ms, 60000 for LCD, 120000 for Matrix, 120000 for LCD_BIG
const unsigned long SCREEN_CYCLE_FREQUENCY = 15500;       //ms, 5000 for LCD, 15500 for Matrix, 15500 for LCD_BIG
const int PIR_PIN = 14;                                   //14 for LCD, 2 for Matrix
const unsigned long PIR_TURN_OFF_TIME = 300000;           //ms, 300000 for LCD, 120000 for Matrix
float PRECISSION_TEMP = 1.0;                              //degrees
float PRECISSION_HUMID = 2.0;                             //percentage
float PRECISSION_AQI = 10.0;                              //value
const long DIM_DURATION = 500;                            // interval for high state (milliseconds) 500 for LCD, 2000 for Matrix
const long NOT_DIM_DURATION = 2000;                       // interval for low state (milliseconds) 2000 for LCD, 500 for Matrix
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
int alarmHrs = 0;                                   //24 hrs format
int alarmMins = 0;                                  //0 to 60
boolean alarmEnabled;
boolean alarmStarted;
boolean isAttention;
fauxmoESP fauxmo;
volatile boolean takeActionToSwitchOnDevice, takeActionToSwitchOffDevice;
volatile int deviceValue = 255;
String lastSentMessage;
unsigned long attention_previousMillis = 0;  // will store last time LED was updated
bool attention_isHigh = false;               // flag to track the state
//First record audio file
//Convert to wav using https://cloudconvert.com
//Compress wav using https://www.freeconvert.com/wav-compressor
//Generate hex using https://tomeko.net/online_tools/file_to_hex.php?lang=en
AudioGeneratorWAV* wav;
AudioFileSourcePROGMEM* file;
AudioOutputI2SNoDAC* out;
static unsigned long last = millis();
static unsigned long startMillis = millis();
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
  fauxmoSetup();
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
  startAudio(welcomeAudio, sizeof(welcomeAudio));
}

void loop() {
  if (isWelcomePlaying) {
    if ((millis() > (startMillis + 7500))) {
      stopAudio();
      isWelcomePlaying = false;
    }
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
  fauxmoLoop();
  checkAndSetAlarm();
  checkAndsendToAlexaAlarmReadings();
  checkAndStartAlarm();
  audioLoopSection();
  displayScreen((isAttention && attention_dim()) || (alarmStarted && attention_dim()));
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
  Serial.println("begining the data fetch now");
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

void checkAndStartAlarm() {
  if (alarmEnabled && hour() == alarmHrs && minute() == alarmMins) {
    if (!alarmStarted) {
      startAudio(alarmAudio, sizeof(alarmAudio));
      alarmStarted = true;
    }
  } else {
    if (alarmStarted) {
      stopAudio();
      alarmStarted = false;
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
  String meessageToSend;
  if (alarmEnabled)
    meessageToSend = String(alarmHrs) + "Hr%20" + String(alarmMins) + "Min";
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

void checkAndSetAlarm() {
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

volatile boolean playAudio;
const unsigned char* globalAudioData = nullptr;
size_t globalDataSize = 0;

void stopAudio() {
  if (wav != nullptr) {
    wav->stop();
    playAudio = false;
    Serial.println("Audio stopped");
  }
}

void startAudio(const unsigned char audioData[], size_t dataSize) {
  stopAudio();
  delete file;
  delete out;
  delete wav;

  file = new AudioFileSourcePROGMEM(audioData, dataSize);
  out = new AudioOutputI2SNoDAC();
  wav = new AudioGeneratorWAV();
  playAudio = true;
  wav->begin(file, out);
  // Save audio data and size to global variables
  globalAudioData = audioData;
  globalDataSize = dataSize;

  Serial.println("Audio triggered");
}

void audioLoopSection() {
  if (wav != nullptr) {
    if (wav->isRunning()) {
      if (!wav->loop()) wav->stop();
    } else {
      if (playAudio && globalAudioData != nullptr) {
        Serial.println("This Audio session is done. Starting new session");
        startAudio(globalAudioData, globalDataSize);
      }
    }
  }
}