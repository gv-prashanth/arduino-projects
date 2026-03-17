/*
================================================================================
 ESP32 / ESP8266 – Atomberg Fan Listener → Alexa Updater (UDP → HTTPS Bridge)
--------------------------------------------------------------------------------
 Listens for UDP broadcasts from Atomberg fans, decodes state, tracks devices,
 and pushes updates to Alexa via HTTPS.

 CONFIGURATION
   SERIAL_LOG    – set false for production (USB power adapter, no computer).
   WIFI_TX_DBM   – WiFi transmit power in dBm.
                   Use 10 for USB-to-computer, 15-20 for USB power adapter.

 LED STATUS (onboard LED, active LOW on ESP8266 GPIO2)
   Solid ON        – WiFi connected, HTTPS working
   Slow blink (1s) – WiFi connected but HTTPS is failing
   Fast blink (¼s) – WiFi disconnected
================================================================================
*/

#define SERIAL_LOG    false
#define WIFI_TX_DBM   10
#define LED_PIN       2
#define HTTPS_FAIL_THRESHOLD 5

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
#include <ArduinoJson.h>

#if SERIAL_LOG
  #define LOG(...) Serial.printf(__VA_ARGS__)
  #define LOGLN(s) Serial.println(s)
#else
  #define LOG(...) do {} while(0)
  #define LOGLN(s) do {} while(0)
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

#if defined(ESP8266)
  BearSSL::WiFiClientSecure secureClient;
#else
  WiFiClientSecure secureClient;
#endif
HTTPClient httpClient;

// ---- Master name table ----
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
  int  lastKnownSpeed;
  uint32_t lastHeartbeat;
};

Device devices[MAX_DEVICES];
uint8_t deviceCount = 0;

// ---- Alexa message queue (ring buffer) ----
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

// ---- HTTPS health tracking ----
uint8_t httpsFailCount = 0;

void resetSecureClient() {
  LOG("[HTTPS] Resetting BearSSL client\n");
  secureClient.stop();
#if defined(ESP8266)
  secureClient.setBufferSizes(512, 512);
#endif
  secureClient.setInsecure();
}

void forceWifiRecovery() {
  LOG("[HTTPS] %d consecutive failures — forcing WiFi recovery\n", httpsFailCount);
  httpsFailCount = 0;
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  resetSecureClient();
  udp.stop();
  udp.begin(UDP_PORT);
}

// ---- LED status ----
enum LedState { LED_SOLID, LED_SLOW_BLINK, LED_FAST_BLINK };
LedState ledState = LED_FAST_BLINK;
uint32_t lastLedToggle = 0;
bool ledOn = false;

void updateLed() {
  uint32_t now = millis();
  switch (ledState) {
    case LED_SOLID:
      if (!ledOn) { digitalWrite(LED_PIN, LOW); ledOn = true; }
      break;
    case LED_SLOW_BLINK:
      if (now - lastLedToggle > 1000) {
        lastLedToggle = now;
        ledOn = !ledOn;
        digitalWrite(LED_PIN, ledOn ? LOW : HIGH);
      }
      break;
    case LED_FAST_BLINK:
      if (now - lastLedToggle > 250) {
        lastLedToggle = now;
        ledOn = !ledOn;
        digitalWrite(LED_PIN, ledOn ? LOW : HIGH);
      }
      break;
  }
}

void setLedState(LedState s) {
  if (ledState != s) {
    ledState = s;
    lastLedToggle = millis();
  }
}

// ---- WiFi watchdog (no reboot — escalating recovery) ----
uint32_t wifiOkAt = 0;
uint32_t lastReconnect = 0;
bool wifiWasOffline = false;

void wifiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiWasOffline) {
      wifiWasOffline = false;
      LOG("[WIFI] Back online — restarting UDP\n");
      udp.stop();
      udp.begin(UDP_PORT);
      resetSecureClient();
    }
    wifiOkAt = millis();

    if (httpsFailCount >= HTTPS_FAIL_THRESHOLD)
      setLedState(LED_SLOW_BLINK);
    else
      setLedState(LED_SOLID);
    return;
  }

  wifiWasOffline = true;
  setLedState(LED_FAST_BLINK);
  uint32_t offline = millis() - wifiOkAt;

  if (offline > 90000 && millis() - lastReconnect > 60000) {
    lastReconnect = millis();
    LOG("[WIFI] Offline %lus — full teardown + reconnect\n", offline / 1000);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
  } else if (offline > 30000 && millis() - lastReconnect > 30000) {
    lastReconnect = millis();
    LOG("[WIFI] Offline %lus — reconnecting\n", offline / 1000);
    WiFi.reconnect();
  }
}

// ---- URL-encode spaces ----
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

// ---- Send ONE message to Alexa ----
void sendToAlexa(const char* name, const char* reading) {
  if (WiFi.status() != WL_CONNECTED) return;

  static char encName[64], encReading[64], url[200];
  urlEncodeSpaces(name, encName, sizeof(encName));
  urlEncodeSpaces(reading, encReading, sizeof(encReading));
  snprintf(url, sizeof(url),
    "https://home-automation.vadrin.com/droid/%s/upsert/intent/%s/reading/%s",
    DROID_ID, encName, encReading);

  LOG("[HTTP] %s\n", url);

  httpClient.setTimeout(5000);
  if (httpClient.begin(secureClient, url)) {
    int code = httpClient.GET();
    if (code >= 200 && code < 300) {
      LOG("[HTTP] %d\n", code);
      httpsFailCount = 0;
    } else {
      LOG("[HTTP] FAIL %d\n", code);
      httpsFailCount++;
      if (httpsFailCount == HTTPS_FAIL_THRESHOLD) {
        forceWifiRecovery();
      }
    }
    httpClient.end();
  } else {
    httpsFailCount++;
    LOG("[HTTP] begin failed (%d consecutive)\n", httpsFailCount);
    if (httpsFailCount == HTTPS_FAIL_THRESHOLD) {
      forceWifiRecovery();
    }
  }
}

bool trySendOneFromQueue() {
  if (qCount == 0) return false;
  sendToAlexa(alexaQ[qHead].name, alexaQ[qHead].reading);
  qHead = (qHead + 1) % QUEUE_SIZE;
  qCount--;
  return true;
}

// ---- Device helpers ----
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
  if (power && speed > 0) snprintf(msg, sizeof(msg), "On, Speed %d.", speed);
  else if (!power && speed > 0) snprintf(msg, sizeof(msg), "On, Speed 0.");
  else if (devices[i].lastKnownSpeed > 0) snprintf(msg, sizeof(msg), "On, Speed %d.", devices[i].lastKnownSpeed);
  else snprintf(msg, sizeof(msg), "On");

  LOG("[DEV] %s %s: %s\n", isNew ? "NEW" : "UPD", name, msg);
  if (notify) enqueueAlexa(name, msg);
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
      if (devices[i].speed > 0 && devices[i].power)
        devices[i].lastKnownSpeed = devices[i].speed;
      devices[i].switchOn = false;
      const char* name = lookupName(devices[i].id);
      LOG("[DEV] OFF %s\n", name);
      enqueueAlexa(name, "Off");
    }
  }
}

void loadDefaults() {
  for (int i = 0; i < MASTER_COUNT; i++) {
    upsertDevice(MASTER[i].id, false, false, 0, true);
    yield();
  }
}

// ==== Setup ====
void setup() {
#if SERIAL_LOG
  Serial.begin(115200);
  delay(2000);
  LOGLN("\n\n========== BOOT ==========");
  #if defined(ESP8266)
    LOG("Reset reason: %s\n", ESP.getResetReason().c_str());
  #endif
  LOG("Free heap   : %u\n", ESP.getFreeHeap());
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  WiFi.setOutputPower(WIFI_TX_DBM);
  WiFi.begin(ssid, password);
#if SERIAL_LOG
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  LOG("\nIP: %s  RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
#else
  while (WiFi.status() != WL_CONNECTED) delay(250);
#endif

  WiFi.setAutoReconnect(true);

#if defined(ESP8266)
  secureClient.setBufferSizes(512, 512);
#endif
  secureClient.setInsecure();

  udp.begin(UDP_PORT);
  loadDefaults();

  wifiOkAt = millis();
  setLedState(LED_SOLID);
  LOG("Heap: %u\nSetup complete.\n\n", ESP.getFreeHeap());
}

// ==== Loop ====
uint32_t lastOffCheck = 0;
uint32_t lastHeapLog  = 0;

void loop() {
  wifiWatchdog();
  updateLed();

  if (millis() - lastHeapLog > 120000) {
    lastHeapLog = millis();
    LOG("[HEAP] %u (queue: %d fails: %d)\n", ESP.getFreeHeap(), qCount, httpsFailCount);
  }

  int sz = udp.parsePacket();
  if (sz > 0) {
    static char buf[1024];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

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
  }

  if (millis() - lastOffCheck > 2000) {
    lastOffCheck = millis();
    checkHeartbeatOff();
  }

  trySendOneFromQueue();

  yield();
}
