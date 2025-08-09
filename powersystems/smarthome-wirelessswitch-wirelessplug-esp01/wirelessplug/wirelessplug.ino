/*
WIFI Switch with Power Stabilization Delay and LED Blink Patterns
  - Timein phase (waiting): Slow blink (500ms ON / 500ms OFF)
  - ON phase: LED steady ON
  - Timeout phase: Fast blink (100ms ON / 100ms OFF)
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Configure below
const char* DEVICENAMENOSPACE = "masterac";
const unsigned long timein = 5000; // Delay in ms before turning ON after receiving first packet
const unsigned long timeout = 5000; // Delay in ms before turning OFF after loosing packet

// Don't touch below lines
const char* ssid = DEVICENAMENOSPACE;
const char* password = "12345678";
const int udpPort = 4210;

WiFiUDP udp;

unsigned long lastPacketTime = 0;
unsigned long firstPacketTime = 0;

// LED & Output pins
const int LED_PIN = 1; // Onboard LED (LOW = ON)
const int RELAY_PIN = 3;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED OFF
  digitalWrite(RELAY_PIN, LOW); // Relay OFF

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    blinkLED(100);
    delay(10);
  }

  udp.begin(udpPort);
}

void loop() {
  // Receive packet
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incoming[10];
    udp.read(incoming, 10);
    lastPacketTime = millis();

    if (firstPacketTime == 0) {
      firstPacketTime = millis(); // Mark first packet time for timein phase
    }
  }

  unsigned long now = millis();

  if (now - lastPacketTime < timeout) {
    // We are in active packet phase
    if (firstPacketTime != 0 && (now - firstPacketTime < timein)) {
      // TIMEIN phase → slow blink
      blinkLED(500);
      digitalWrite(RELAY_PIN, LOW);
    } else {
      // ON phase → LED steady ON
      digitalWrite(LED_PIN, LOW);  // LED ON
      digitalWrite(RELAY_PIN, HIGH); // Relay ON
    }
  } else {
    // TIMEOUT phase → fast blink
    blinkLED(100);
    digitalWrite(RELAY_PIN, LOW);
    firstPacketTime = 0; // Reset for next ON cycle
  }
}

// Blink LED with given interval (ms ON/OFF)
void blinkLED(unsigned long interval) {
  static unsigned long lastToggle = 0;
  static bool ledState = false;

  if (millis() - lastToggle >= interval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH); // LOW = ON for onboard LED
    lastToggle = millis();
  }
}
