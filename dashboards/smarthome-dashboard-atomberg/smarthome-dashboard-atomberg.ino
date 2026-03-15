/*
================================================================================
 ESP32 / ESP8266 – Atomberg Fan Listener → Alexa Updater (UDP → HTTPS Bridge)
================================================================================
 FEATURE FLAGS — disable all, then enable one at a time to isolate crashes.
 Start with all false. Upload. Verify stability. Enable one. Upload. Repeat.
*/

#define FEATURE_UDP_DECODE    true   // hex decode + device tracking
#define FEATURE_HTTPS         true   // send updates to Alexa via HTTPS
#define FEATURE_HEARTBEAT_OFF true   // mark devices Off after 10s no heartbeat
#define FEATURE_WIFI_WATCHDOG true   // reconnect / reboot on WiFi loss

#if defined(ESP32)
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <WiFiClientSecure.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClientSecureBearSSL.h>
#endif
#include <WiFiUdp.h>
#if FEATURE_UDP_DECODE
  #include <ArduinoJson.h>
#endif

#define UDP_PORT     5625
#define MAX_DEVICES  20
#define MAX_ID_LEN   20
#define MAX_NAME_LEN 32
#define QUEUE_SIZE   10

const char* ssid     = "XXXX";
const char* password = "YYYY";
const char* DROID_ID = "ABCD";

WiFiUDP udp;

// ---- HTTPS globals (only when HTTPS enabled) ----
#if FEATURE_HTTPS
  #if defined(ESP8266)
    BearSSL::WiFiClientSecure secureClient;
  #else
    WiFiClientSecure secureClient;
  #endif
  HTTPClient httpClient;
#endif

// ---- Master name table ----
#if FEATURE_UDP_DECODE
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

// ---- Device registry ----
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
#endif

// ---- Alexa message queue ----
#if FEATURE_HTTPS
struct AlexaMsg {
  char name[MAX_NAME_LEN];
  char reading[48];
};

AlexaMsg alexaQ[QUEUE_SIZE];
uint8_t qHead = 0, qCount = 0;

void enqueueAlexa(const char* name, const char* reading) {
  if (qCount >= QUEUE_SIZE) {
    qHead = (qHead + 1) % QUEUE_SIZE;
    qCount--;
  }
  int idx = (qHead + qCount) % QUEUE_SIZE;
  snprintf(alexaQ[idx].name, sizeof(alexaQ[idx].name), "%s", name);
  snprintf(alexaQ[idx].reading, sizeof(alexaQ[idx].reading), "%s", reading);
  qCount++;
}

void urlEncodeSpaces(const char* in, char* out, int outSize) {
  int j = 0;
  for (int i = 0; in[i] && j < outSize - 4; i++) {
    if (in[i] == ' ') {
      out[j++] = '%'; out[j++] = '2'; out[j++] = '0';
    } else {
      out[j++] = in[i];
    }
  }
  out[j] = '\0';
}

void sendToAlexa(const char* name, const char* reading) {
  if (WiFi.status() != WL_CONNECTED) return;

  static char encName[64], encReading[64], url[200];
  urlEncodeSpaces(name, encName, sizeof(encName));
  urlEncodeSpaces(reading, encReading, sizeof(encReading));
  snprintf(url, sizeof(url),
    "https://home-automation.vadrin.com/droid/%s/upsert/intent/%s/reading/%s",
    DROID_ID, encName, encReading);

  Serial.printf("[HTTP] %s\n", url);

  httpClient.setTimeout(5000);
  if (httpClient.begin(secureClient, url)) {
    int code = httpClient.GET();
    Serial.printf("[HTTP] %d\n", code);
    httpClient.end();
  }
}

bool trySendOneFromQueue() {
  if (qCount == 0) return false;
  sendToAlexa(alexaQ[qHead].name, alexaQ[qHead].reading);
  qHead = (qHead + 1) % QUEUE_SIZE;
  qCount--;
  return true;
}
#endif

// ---- WiFi watchdog ----
#if FEATURE_WIFI_WATCHDOG
uint32_t wifiOkAt = 0;
uint32_t lastReconnect = 0;

void wifiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiOkAt = millis();
    return;
  }
  uint32_t offline = millis() - wifiOkAt;
  if (offline > 30000 && millis() - lastReconnect > 30000) {
    lastReconnect = millis();
    Serial.printf("[WIFI] Offline %lus — reconnecting\n", offline / 1000);
    WiFi.reconnect();
  }
  if (offline > 90000) {
    Serial.println("[WIFI] Offline >90s — reboot");
    delay(500);
    ESP.restart();
  }
}
#endif

// ---- Device helpers ----
#if FEATURE_UDP_DECODE
const char* lookupName(const char* id) {
  for (int i = 0; i < MASTER_COUNT; i++)
    if (strcmp(MASTER[i].id, id) == 0) return MASTER[i].name;
  static char temp[MAX_ID_LEN];
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
  static char temp[MAX_ID_LEN];
  for (int i = 0; i < deviceCount; i++) {
    snprintf(temp, sizeof(temp), "%s_R1", devices[i].id);
    if (strcmp(id, temp) == 0) return i;
  }
  return -1;
}

int upsertDevice(const char* id, bool power, bool led, int speed, bool notify) {
  if (!id) return -1;
  int i = findDevice(id);
  bool isNew = (i < 0);
  if (isNew) {
    if (deviceCount >= MAX_DEVICES) return -1;
    i = deviceCount++;
    snprintf(devices[i].id, sizeof(devices[i].id), "%s", id);
  }
  devices[i].power    = power;
  devices[i].switchOn = true;
  devices[i].led      = led;
  devices[i].speed    = speed;
  devices[i].lastHeartbeat = millis();

  const char* name = lookupName(id);
  static char msg[48];
  if (power && speed > 0) snprintf(msg, sizeof(msg), "On. Speed %d.", speed);
  else if (!power && speed > 0) snprintf(msg, sizeof(msg), "On. Standby.");
  else snprintf(msg, sizeof(msg), "On");

  Serial.printf("[DEV] %s %s: %s\n", isNew ? "NEW" : "UPD", name, msg);
#if FEATURE_HTTPS
  if (notify) enqueueAlexa(name, msg);
#endif
  return i;
}

void processFanEvent(const char* hex, int hexLen) {
  if (hexLen < 2 || hexLen > 1024) return;
  static char json[512];
  memset(json, 0, sizeof(json));
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
  static char numBuf[16];
  memset(numBuf, 0, sizeof(numBuf));
  if (pos > 0 && pos < (int)sizeof(numBuf))
    snprintf(numBuf, sizeof(numBuf), "%.*s", pos, state);
  uint32_t enc = strtoul(numBuf, NULL, 10);
  upsertDevice(id, (enc & 0x10) != 0, (enc & 0x20) != 0, enc & 0x07, true);
}

void checkHeartbeatOff() {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].switchOn && (millis() - devices[i].lastHeartbeat > 10000)) {
      devices[i].switchOn = false;
      const char* name = lookupName(devices[i].id);
      Serial.printf("[DEV] OFF %s\n", name);
#if FEATURE_HTTPS
      enqueueAlexa(name, "Off");
#endif
    }
  }
}

void loadDefaults() {
  for (int i = 0; i < MASTER_COUNT; i++) {
    upsertDevice(MASTER[i].id, false, false, 0, false);
    yield();
  }
}
#endif

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n========== BOOT ==========");
  Serial.printf("Features: UDP_DECODE=%d HTTPS=%d HB_OFF=%d WIFI_WD=%d\n",
    FEATURE_UDP_DECODE, FEATURE_HTTPS, FEATURE_HEARTBEAT_OFF, FEATURE_WIFI_WATCHDOG);
#if defined(ESP8266)
  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());
  Serial.printf("Reset info  : %s\n", ESP.getResetInfo().c_str());
#endif
  Serial.printf("Free heap   : %u\n", ESP.getFreeHeap());

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
#if defined(ESP32)
  WiFi.setSleep(false);
#endif
  WiFi.begin(ssid, password);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

#if FEATURE_HTTPS
  #if defined(ESP8266)
    secureClient.setBufferSizes(512, 512);
  #endif
  secureClient.setInsecure();
#endif

  udp.begin(UDP_PORT);

#if FEATURE_UDP_DECODE
  loadDefaults();
#endif

#if FEATURE_WIFI_WATCHDOG
  wifiOkAt = millis();
#endif

  Serial.printf("Heap: %u\nReady.\n\n", ESP.getFreeHeap());
}

// ==== Loop ====
uint32_t lastOffCheck = 0;
uint32_t lastHeapLog  = 0;

void loop() {
#if FEATURE_WIFI_WATCHDOG
  wifiWatchdog();
#endif

  if (millis() - lastHeapLog > 120000) {
    lastHeapLog = millis();
    Serial.printf("[HEAP] %u\n", ESP.getFreeHeap());
  }

  int sz = udp.parsePacket();
  if (sz > 0) {
    static char buf[1024];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

#if FEATURE_UDP_DECODE
    bool isHex = (len > 2);
    for (int i = 0; i < len && isHex; i++)
      if (!isxdigit((unsigned char)buf[i])) isHex = false;

    if (isHex) {
      processFanEvent(buf, len);
    } else {
      int idx = findDevice(buf);
      if (idx >= 0) {
        devices[idx].lastHeartbeat = millis();
        if (!devices[idx].switchOn)
          upsertDevice(buf, false, false, 0, true);
      } else if (len > 0 && len < MAX_ID_LEN) {
        upsertDevice(buf, false, false, 0, true);
      }
    }
#else
    Serial.printf("[UDP] %d bytes: %.40s\n", len, buf);
#endif
  }

#if FEATURE_UDP_DECODE && FEATURE_HEARTBEAT_OFF
  if (millis() - lastOffCheck > 2000) {
    lastOffCheck = millis();
    checkHeartbeatOff();
  }
#endif

#if FEATURE_HTTPS
  trySendOneFromQueue();
#endif

  yield();
}
