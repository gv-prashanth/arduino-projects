/*
  Test 2b — WiFi without WiFi.mode(WIFI_STA), matching original code.
*/

#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif

const char* ssid     = "XXXX";
const char* password = "YYYY";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Test 2b: WiFi (no mode set)");

  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Heap: %u\nReady.\n", ESP.getFreeHeap());
}

uint32_t counter = 0;

void loop() {
  Serial.printf("alive %lu heap=%u\n", counter++, ESP.getFreeHeap());
  delay(2000);
}
