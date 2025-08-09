/*
WIFI Switch
  The concept:
   Turns ON and OFF an AC  Through WIFI Remotely.
   Using ESP8266  as Transmitter and Receiver modules for WIFI wireless link, Turning power ON to AC as long as Power available 
   to TX module. RX module turns on  connected Solid State Relay mounted on Power supply unit.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* DEVICENAMENOSPACE = "masterac";

//Dont touch below lines
const char* ssid = DEVICENAMENOSPACE;
const char* password = "12345678"; //Default no need to change
const IPAddress clientIP(192, 168, 4, 2);  // Default no need to change. IP of Rx is 2 since the plug is second device
const int udpPort = 4210;
const int heartBeatTime = 500; //ms

WiFiUDP udp;

void setup() {
  WiFi.softAP(ssid, password);
  delay(heartBeatTime);  // allow AP to initialize
}

void loop() {
  udp.beginPacket(clientIP, udpPort);
  udp.write("ON");
  udp.endPacket();
  delay(heartBeatTime);  // heartbeat every half seconds
}