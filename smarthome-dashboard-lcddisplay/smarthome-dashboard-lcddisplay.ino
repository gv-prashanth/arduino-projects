#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BearSSLHelpers.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Configurations
const char* ssid = "XXX";
const char* password = "YYY";
const String droid = "ZZZ";
const unsigned long PAYLOAD_SAMPLING_FREQUENCY = 60000;  //ms
const unsigned long SCREEN_CYCLE_FREQUENCY = 5000;       //ms
const int SCREEN_WIDTH = 20;                             //characters
const int SCREEN_HEIGHT = 4;                             //rows
const int PAYLOAD_START_ROW = 2;                         //index. Starts from 0.
const int PIR_PIN = 14;                                  // PIR sensor input pin
const unsigned long PIR_TURN_OFF_TIME = 30000;           //ms

// Dont touch below
const String serverAddress = "https://home-automation.vadrin.com";  // Note the "https://" prefix
const String endpoint = "/droid/" + droid + "/intents";
LiquidCrystal_I2C lcd(0x27, SCREEN_WIDTH, SCREEN_HEIGHT);  // Set the LCD I2C address
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

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);  //Setup the PIR
  setupLCD();
  setupWifi();
  motionDetectedRecently = true;
  //Lets fetch and parse once to be ready to display immediatly.
  fetchPayload(); 
  parsePayload();
}

void loop() {
  unsigned long currentTime = millis();

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
      displayMessage(String(globalDataEntries[indexToDisplay].key) + String(" is ") + String(globalDataEntries[indexToDisplay].deviceReading));
      lastScreenChangeTime = currentTime;
    }
  } else {
    //switch off everything by Clearing the display and turn off the backlight
    lcd.clear();
    lcd.noBacklight();
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
}

void setupLCD() {
  lcd.init();  // Initialize the LCD
  displayMessage("Please Wait...");
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

void displayMessage(String str) {

  lcd.clear();
  lcd.backlight();  // Turn on the backlight

  //Print header
  String customHeader = droid + String(" HOME");
  lcd.setCursor((int)((SCREEN_WIDTH - customHeader.length()) / 2), 0);
  lcd.print(customHeader);

  //Print str
  Serial.println(str);
  if (str.length() > (SCREEN_HEIGHT - PAYLOAD_START_ROW) * SCREEN_WIDTH)
    str = str.substring(0, (SCREEN_HEIGHT - PAYLOAD_START_ROW) * SCREEN_WIDTH);  // Ensure the str doesn't exceed the line limit

  int maxLineLength = SCREEN_WIDTH;
  int startLine = PAYLOAD_START_ROW;
  int endLine = SCREEN_HEIGHT - 1;
  String currentLine = "";
  String words[50];  // Assuming a maximum of 50 words
  int wordCount = 0;

  for (int i = 0; i < str.length(); i++) {
    char currentChar = str.charAt(i);
    if (currentChar != ' ') {
      currentLine += currentChar;
    } else {
      words[wordCount] = currentLine;
      wordCount++;
      currentLine = "";
    }
  }
  words[wordCount] = currentLine;
  wordCount++;

  // Initialize the current line with the first word
  currentLine = words[0];

  for (int i = 1; i < wordCount; i++) {
    String word = words[i];
    if (currentLine.isEmpty() || currentLine.length() + word.length() + 1 <= maxLineLength) {
      if (!currentLine.isEmpty()) {
        currentLine += ' ';  // Add a space between words
      }
      currentLine += word;
    } else {
      if (startLine < endLine) {
        // Display the current line on the LCD
        lcd.setCursor(0, startLine);
        lcd.print(currentLine);
        startLine++;
        currentLine = word;
      } else {
        // Maximum lines reached; exit the loop
        break;
      }
    }
  }

  // Display any remaining content
  if (startLine <= endLine) {
    lcd.setCursor(0, startLine);
    lcd.print(currentLine);
  }
}
