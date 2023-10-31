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
const unsigned long SCREEN_CYCLE_FREQUENCY = 3000;       //ms

// Dont touch below
const String serverAddress = "https://home-automation.vadrin.com";  // Note the "https://" prefix
String endpoint = "/droid/"+droid+"/intents";
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD I2C address
String payload;
int indexToDisplay = 0;
unsigned long lastFetchTime, lastScreenChangeTime;
struct SensorData {
  int jsonIndex;
  String key;
  String deviceReading;
  String readingTime;
};
std::vector<SensorData> globalDataEntries;  // Global vector to store the data
boolean firstTime;

void setup() {
  Serial.begin(115200);
  setupLCD();
  setupWifi();
  firstTime = true;
}

void loop() {
  unsigned long currentTime = millis();
  if (firstTime || (currentTime - lastFetchTime > PAYLOAD_SAMPLING_FREQUENCY)) {
    Serial.println(".....START.....");
    fetchPayload();
    parsePayload();
    lastFetchTime = currentTime;
    Serial.println("......END......");
  }
  if (firstTime || (currentTime - lastScreenChangeTime > SCREEN_CYCLE_FREQUENCY)) {
    indexToDisplay++;
    if (indexToDisplay >= globalDataEntries.size()) {
      indexToDisplay = 0;
    }
    printPayload();
    firstTime = false;
    lastScreenChangeTime = currentTime;
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
  lcd.init();       // Initialize the LCD
  lcd.backlight();  // Turn on the backlight
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
  DynamicJsonDocument doc(1024);  // Adjust the size based on your JSON data size

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

void printPayload() {
  lcd.clear();
  //Print header
  String customHeader = droid + String(" HOME");
  lcd.setCursor((int)((20-customHeader.length())/2), 0);
  lcd.print(customHeader);

  SensorData entry = globalDataEntries[indexToDisplay];
  Serial.print("Key: ");
  Serial.println(entry.key);
  Serial.print("Device Reading: ");
  Serial.println(entry.deviceReading);
  Serial.print("Reading Time: ");
  Serial.println(entry.readingTime);
  String message = String(entry.key) + String(" is ") + String(entry.deviceReading);
  // Ensure the message doesn't exceed the 4-line limit
  if (message.length() > 40) {
    message = message.substring(0, 40);
  }
  // Display the message on the LCD with a maximum line length of 20 characters and a maximum of 4 lines
  displayStringOnLCD(message, 20, 2, 3);
}

void displayStringOnLCD(String str, int maxLineLength, int startLine, int endLine) {
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
