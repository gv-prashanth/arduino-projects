#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BearSSLHelpers.h>
#include <ArduinoJson.h>

// Replace with your network credentials
const char* ssid = "XXX";
const char* password = "YYY";
const String droid = "ZZZ";

// Replace with the server's address and the endpoint you want to access
const String serverAddress = "https://home-automation.vadrin.com";  // Note the "https://" prefix
const String endpoint = "/droid/"+droid+"/intents";

void setup() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {

    Serial.println("Fetching from web");

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

    String payload = http.getString();
    //Serial.println("Response payload:");
    //Serial.println(payload);


  DynamicJsonDocument doc(1024); // Adjust the size based on your JSON data size

  // Deserialize the JSON data
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("Error parsing JSON: ");
    Serial.println(error.c_str());
  } else {
    // Iterate through each entry and print it
    for (JsonPair kv : doc.as<JsonObject>()) {
      const char* key = kv.key().c_str();
      const char* deviceReading = kv.value()["deviceReading"];
      const char* readingTime = kv.value()["readingTime"];

      Serial.print(key);
      Serial.print(": ");
      Serial.println(deviceReading);
      //Serial.print("Reading Time: ");
      //Serial.println(readingTime);
      //Serial.println();
    }
  }


  } else {
    Serial.print("Error on HTTPS request: ");
    Serial.println(httpResponseCode);
  }

  http.end();

  // Wait for 10 seconds before making the next request
  delay(10000);
}
