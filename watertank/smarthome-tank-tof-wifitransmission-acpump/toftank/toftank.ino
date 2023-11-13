/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank.
   Using ESP8266 WIFI used as Transmitter and Receiver modules for wireless link, along with HC-SR04 ultrasonic sensor sense the water level reading.
   ESP as logic controller to drive a sumpMotor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank is connected to a TOF distance sensor (Ultrasonic sensor)
   sumpMotorTriggerPin driver attached to ESP8266 receiver pin 5  with 10k resistor to ground
*/

#include <ESP8266WiFi.h>
#include <NewPingESP8266.h>

#define TRIGGER_PIN  0  // ESP pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // ESP pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

const int RETRY_ATTEMPTS = 20;
const int sleepTimeSeconds = 8;// Configure deep sleep in between measurements

//Functional Configurations
const char* ssid = "ESP12E";
const char* password = "0607252609";
const char* host = "192.168.11.4"; // as specified in server

//Dont touch below stuff
NewPingESP8266 sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPingESP8266 setup of pins and maximum distance.
WiFiClient client;

void setup()
{
  //Serial.begin(74880);
  // Connect to the server
  connectToServer();

  //send the message to server
  if (WiFi.status() == WL_CONNECTED) {
    //get average distance
    float distance = getMeanDistance();
    sendMessageToServer(distance);
    Serial.print(distance); Serial.println(" cm"); delay(100);
  }

  //sleep for 8 sec
  Serial.println("ESP8266 in sleep mode");
  ESP.deepSleep(sleepTimeSeconds * 1e6);
}

void loop()
{

}

float getMeanDistance() {
  float toAverageDistance = 0;
  int validCounts = 0;
  for (int i = 0; i < 5; i++) {
    float thisDist = sonar.ping_cm();
    if (thisDist != 0) {
      toAverageDistance = toAverageDistance + thisDist;
      validCounts++;
    }
    delay(50);
  }
  if (validCounts > 0)
    return toAverageDistance / validCounts;
  else
    return 0;
}

void connectToServer() {
  int i = 0;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && i < RETRY_ATTEMPTS) {
    i++;
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED){
    Serial.print("IP Address (AP): "); Serial.println(WiFi.localIP());
  }else
    Serial.println("Unable to reach the sump in this session. Will try again later.");
}

void sendMessageToServer(float distance) {
  if (client.connect(host, 80)) {
    String url = "/update?value=";
    url += String(distance);
    client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host +  "\r\n" +
                 "Connection: keep-alive\r\n\r\n"); // minimum set of required URL headers
    delay(10);
    // Read all the lines of the response and print them to Serial
    Serial.println("Response: ");
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
  } else {
    Serial.println("Failed to establish connection");
  }
}
