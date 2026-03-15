/*
  Phase 2 — WiFi + UDP + device tracking + hex decode (NO HTTPS)
  All state changes printed to Serial only.
*/

#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#define UDP_PORT     5625
#define MAX_DEVICES  20
#define MAX_ID_LEN   20
#define MAX_NAME_LEN 32

const char* ssid     = "XXXX";
const char* password = "YYYY";
const char* DROID_ID = "ABCD";

WiFiUDP udp;

struct MasterName {
  char id[MAX_ID_LEN];
  char name[MAX_NAME_LEN];
};

const MasterName MASTER[] = {
  { "24587c849608", "Master Fan" },
  { "3c8427853c00", "Kids Fan" },
  { "3c84278a9f84", "Parents Fan" },
  { "80659934c6b8", "Living Fan" },
  { "3c842783d658", "Dining Fan" }
};
const int MASTER_COUNT = sizeof(MASTER) / sizeof(MASTER[0]);

struct Device {
  char id[MAX_ID_LEN];
  bool power;
  bool switchOn;
  bool led;
  int  speed;
  uint32_t lastHeartbeat;
};

Device devices[MAX_DEVICES];
uint8_t deviceCount = 0;

const char* lookupName(const char* id) {
  for (int i = 0; i < MASTER_COUNT; i++) {
    if (strcmp(MASTER[i].id, id) == 0) return MASTER[i].name;
  }
  char temp[MAX_ID_LEN];
  for (int i = 0; i < MASTER_COUNT; i++) {
    snprintf(temp, sizeof(temp), "%s_R1", MASTER[i].id);
    if (strcmp(temp, id) == 0) return MASTER[i].name;
  }
  return "Unknown Fan";
}

int findDevice(const char* id) {
  if (!id) return -1;
  for (int i = 0; i < deviceCount; i++)
    if (strcmp(id, devices[i].id) == 0) return i;
  char temp[MAX_ID_LEN];
  for (int i = 0; i < deviceCount; i++) {
    snprintf(temp, sizeof(temp), "%s_R1", devices[i].id);
    if (strcmp(id, temp) == 0) return i;
  }
  return -1;
}

int upsertDevice(const char* id, bool power, bool led, int speed) {
  if (!id) return -1;
  int i = findDevice(id);
  bool isNew = (i < 0);
  if (isNew) {
    if (deviceCount >= MAX_DEVICES) {
      Serial.printf("[DEV] Table full — ignoring %s\n", id);
      return -1;
    }
    i = deviceCount++;
    snprintf(devices[i].id, sizeof(devices[i].id), "%s", id);
  }
  devices[i].power     = power;
  devices[i].switchOn  = true;
  devices[i].led       = led;
  devices[i].speed     = speed;
  devices[i].lastHeartbeat = millis();

  const char* name = lookupName(id);
  if (power && speed > 0)
    Serial.printf("[DEV] %s → %s: On. Speed %d.\n", isNew ? "NEW" : "UPD", name, speed);
  else if (!power && speed > 0)
    Serial.printf("[DEV] %s → %s: On. Standby.\n", isNew ? "NEW" : "UPD", name);
  else
    Serial.printf("[DEV] %s → %s: On\n", isNew ? "NEW" : "UPD", name);

  return i;
}

void processFanEvent(const char* hex, int hexLen) {
  if (hexLen < 2 || hexLen > 1024) return;

  char json[512] = {0};
  for (int i = 0, j = 0; i < hexLen && j < 511; i += 2, j++) {
    char h[3] = { hex[i], hex[i + 1], 0 };
    json[j] = (char)strtol(h, NULL, 16);
  }

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, json)) return;

  const char* id    = doc["device_id"];
  const char* state = doc["state_string"];
  if (!id || !state) return;

  const char* comma = strchr(state, ',');
  int pos = comma ? (int)(comma - state) : (int)strlen(state);
  char numBuf[16] = {0};
  if (pos > 0 && pos < (int)sizeof(numBuf))
    snprintf(numBuf, sizeof(numBuf), "%.*s", pos, state);
  uint32_t enc = strtoul(numBuf, NULL, 10);

  upsertDevice(id, (enc & 0x10) != 0, (enc & 0x20) != 0, enc & 0x07);
}

void checkHeartbeatOff() {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].switchOn && (millis() - devices[i].lastHeartbeat > 10000)) {
      devices[i].switchOn = false;
      Serial.printf("[DEV] OFF → %s\n", lookupName(devices[i].id));
    }
  }
}

void loadDefaults() {
  for (int i = 0; i < MASTER_COUNT; i++) {
    upsertDevice(MASTER[i].id, false, false, 0);
    yield();
  }
  Serial.printf("Loaded %d defaults.\n", MASTER_COUNT);
}

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

  loadDefaults();
  Serial.printf("Heap: %u\nPhase 2 ready.\n\n", ESP.getFreeHeap());
}

uint32_t lastOffCheck = 0;

void loop() {
  int sz = udp.parsePacket();
  if (sz > 0) {
    char buf[1024];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

    bool isHex = (len > 0);
    for (int i = 0; i < len && isHex; i++)
      if (!isxdigit((unsigned char)buf[i])) isHex = false;

    if (isHex && len > 2) {
      processFanEvent(buf, len);
    } else {
      int idx = findDevice(buf);
      if (idx >= 0) {
        devices[idx].lastHeartbeat = millis();
        if (!devices[idx].switchOn)
          upsertDevice(buf, false, false, 0);
      } else if (len > 0 && len < MAX_ID_LEN) {
        upsertDevice(buf, false, false, 0);
      }
    }
  }

  if (millis() - lastOffCheck > 2000) {
    lastOffCheck = millis();
    checkHeartbeatOff();
  }

  yield();
}
