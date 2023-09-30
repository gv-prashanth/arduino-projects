#include <ESP8266WiFi.h>
#include <espnow.h>

#include <NewPingESP8266.h>

#define TRIGGER_PIN 1     // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN    3     // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 150  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

WiFiClient client;


ADC_MODE(ADC_VCC);

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xc4, 0x5b, 0xbe, 0x65, 0x14, 0xd4};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float f;
  float vcc;
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;
unsigned long timerDelay = 2000;  // send readings timer

NewPingESP8266 sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPingESP8266 setup of pins and maximum distance.


// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
  }
  else {
    Serial.println("Delivery fail");
  }
}

void setup() {

  // Init Serial Monitor
  Serial.begin(74880);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {


  if ((millis() - lastTime) > timerDelay) {

    // Set values to send
    myData.f = (getMeanDistance());
    myData.vcc = (ESP.getVcc() / 1000.00);
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    lastTime = millis();
  }
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
    return -1;
}