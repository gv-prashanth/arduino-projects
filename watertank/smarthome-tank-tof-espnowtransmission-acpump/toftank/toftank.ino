#include <ESP8266WiFi.h>
#include <espnow.h>

ADC_MODE(ADC_VCC);

const int sleepTimeSeconds = 15;  // Configure deep sleep in between measurements

#include <NewPingESP8266.h>

#define TRIGGER_PIN 1     // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 3        // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 150  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPingESP8266 sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPingESP8266 setup of pins and maximum distance.

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0xc4, 0x5b, 0xbe, 0x65, 0x14, 0xd4 };

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int id;
  float Dist;
  float Vcc;
  int readingId;
} struct_message;

// Create a struct_message called test to store variables to be sent
struct_message SensorData;

unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 10000;       // Interval at which to publish sensor readings
unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "GTS";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("\r\nLast Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
  } else {
    Serial.println("Delivery fail");
  }
}

void setup() {
  // Init Serial Monitor
  //Serial.begin(74880);

  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial);  // Uncomment to verify channel number before
  wifi_promiscuous_enable(1);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(0);
  WiFi.printDiag(Serial);  // Uncomment to verify channel change after

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  // Once ESPNow is successfully init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // Set values to send
    SensorData.id = BOARD_ID;
    float meanDistance = getMeanDistance();
    SensorData.Dist = (meanDistance);
    SensorData.Vcc = (ESP.getVcc() / 1000.00);
    SensorData.readingId = readingId++;

    // Send message via ESP-NOW
    esp_now_send(0, (uint8_t *)&SensorData, sizeof(SensorData));

    Serial.println(ESP.getVcc() / 1000.00);
    Serial.println(meanDistance);

    //ESP Deep Sleep Mode
    //Serial.println("ESP8266 in sleep mode");
    ESP.deepSleep(sleepTimeSeconds * 1e6);
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
