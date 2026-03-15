/*
  Test 2c — WiFi with reduced TX power (10 dBm instead of default 20.5 dBm).
  This reduces peak current draw from ~300mA to ~100mA, preventing USB voltage dips.
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
  Serial.println("Test 2c: WiFi reduced TX power");

  WiFi.setOutputPower(10);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.printf("\nIP: %s  RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  Serial.printf("Heap: %u\nReady.\n", ESP.getFreeHeap());
}

uint32_t counter = 0;

void loop() {
  Serial.printf("alive %lu heap=%u rssi=%d\n", counter++, ESP.getFreeHeap(), WiFi.RSSI());
  delay(2000);
}
