/*
  Phase 1 — WiFi + UDP + Serial
  Bare minimum to verify ESP8266 stability.
*/

#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>

#define UDP_PORT 5625

const char* ssid     = "XXXX";
const char* password = "YYYY";

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n========== BOOT ==========");
#if defined(ESP8266)
  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());
#endif
  Serial.printf("Free heap: %u\n", ESP.getFreeHeap());

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("UDP port %d\n", UDP_PORT);
  Serial.println("Phase 1 ready.\n");
}

void loop() {
  int sz = udp.parsePacket();
  if (sz > 0) {
    char buf[512];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';
    Serial.printf("[UDP] %d bytes: %s\n", len, buf);
  }
  yield();
}
