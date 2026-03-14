/*
================================================================================
 ESP32 / ESP8266 – Atomberg Fan Listener → Alexa Updater (UDP → HTTPS Bridge)
--------------------------------------------------------------------------------
This firmware listens for UDP broadcasts from Atomberg (and similar) fans,
decodes their state, maintains an in-memory device registry, and publishes the
current status to an Alexa-compatible backend via HTTPS.

SUPPORTED BOARDS
  • ESP32
  • ESP8266 (libraries auto-selected)

MAIN WORKFLOW
  1. Connects to WiFi.
  2. Listens on UDP_PORT for incoming packets.
     - Hex payloads  → decoded as JSON → fan state extracted.
     - Plain payload → treated as heartbeat (device presence).
  3. Tracks devices in memory (power, LED, speed, last heartbeat).
  4. Builds human-friendly text messages (ex: "On. Speed 3.", "Off").
  5. URL-encodes values and pushes updates over HTTPS to:
        https://home-automation.vadrin.com/droid/<DROID_ID>/...

  Devices that stop sending heartbeats are automatically marked "Off".

BUILT-IN SAFETY FEATURES
  • WiFi watchdog → reboots device if WiFi remains offline too long.
  • Default device pre-load ensures Alexa has initial values on boot.
  • Heartbeat-based switch detection to avoid stale states.
  • Hex decoding and JSON failure handling.

TUNABLE PARAMETERS (IMPORTANT)
  WIFI CREDENTIALS
    const char* ssid        – WiFi SSID
    const char* password    – WiFi password

  CLOUD IDENTIFIER
    const char* DROID_ID    – Unique backend device identifier

  NETWORK
    UDP_PORT                – Port where fan broadcasts are received

  WIFI WATCHDOG
    WIFI_TIMEOUT_REBOOT     – Max allowed WiFi downtime before reboot (ms)

  DEVICE LIMITS
    MAX_DEVICES             – Maximum number of tracked devices
    MAX_ID_LEN              – Length of device ID strings
    MAX_NAME_LEN            – Length of device display names

  HEARTBEAT OFF TIMEOUT
    (inside upsertDevice_Off)
    10,000 ms → time with no heartbeat before marking device Off

INTERNAL TABLES
  masterNames[] maps known hardware IDs → friendly names
  devices[]     maintains runtime device state cache

NOTE
  TLS uses "setInsecure()" — acceptable for labs/testing but not ideal for
  production. Replace with certificate pinning if security is critical.

================================================================================
*/

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

#define MAX_DEVICES 20
#define MAX_ID_LEN 20
#define MAX_NAME_LEN 32
#define UDP_PORT 5625

const char* ssid = "XXXX";
const char* password = "YYYY";
const char* DROID_ID = "ABCD";

WiFiUDP udp;
#if defined(ESP8266)
  static BearSSL::WiFiClientSecure client;
#else
  static WiFiClientSecure client;
#endif
static HTTPClient https;

struct MasterDeviceName {
  const char id[MAX_ID_LEN];
  const char name[MAX_NAME_LEN];
};

MasterDeviceName masterNames[] = {
  { "24587c849608", "Master Fan" },
  { "3c8427853c00", "Kids Fan" },
  { "3c84278a9f84", "Parents Fan" },
  { "80659934c6b8", "Living Fan" },
  { "3c842783d658", "Dining Fan" }
};

struct DeviceInfo {
  char deviceId[MAX_ID_LEN];
  bool power;
  bool switchOn;
  bool led;
  int speed;
  uint32_t lastHeartbeat;
};

DeviceInfo devices[MAX_DEVICES];
uint8_t deviceCount = 0;

// WIFI WATCHDOG SETTINGS
uint32_t wifiLastConnected = 0;
const uint32_t WIFI_RECONNECT_INTERVAL = 30000;  // nudge reconnect after 30s offline
const uint32_t WIFI_TIMEOUT_REBOOT     = 90000;  // 90s offline → last-resort reboot
const uint32_t HEAP_PRINT_TIME         = 120000;  // 2 minutes
uint32_t lastReconnectAttempt = 0;
uint32_t lastWifiLog = 0;

void wifiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiLastConnected = millis();
    return;
  }

  uint32_t offlineDuration = millis() - wifiLastConnected;

  if (millis() - lastWifiLog > 5000) {
    lastWifiLog = millis();
    Serial.printf("[WIFI] Disconnected for %lu ms (heap: %u)\n",
                  (unsigned long)offlineDuration, ESP.getFreeHeap());
  }

  if (offlineDuration > WIFI_RECONNECT_INTERVAL &&
      millis() - lastReconnectAttempt > WIFI_RECONNECT_INTERVAL) {
    lastReconnectAttempt = millis();
    Serial.println("[WIFI] Auto-reconnect may have stalled — nudging...");
    WiFi.reconnect();
  }

  if (offlineDuration > WIFI_TIMEOUT_REBOOT) {
    Serial.println("\n[FAIL] WiFi offline too long → Restarting ESP...");
    delay(500);
    ESP.restart();
  }
}

/******************************** UTIL *********************************/
void getDeviceName(const char* id, char* out, size_t outSize) {
  for (int i = 0; i < (int)(sizeof(masterNames) / sizeof(masterNames[0])); i++) {
    if (strcmp(masterNames[i].id, id) == 0) {
      snprintf(out, outSize, "%s", masterNames[i].name);
      return;
    }
  }
  char temp[MAX_ID_LEN];
  for (int i = 0; i < (int)(sizeof(masterNames) / sizeof(masterNames[0])); i++) {
    snprintf(temp, sizeof(temp), "%s_R1", masterNames[i].id);
    if (strcmp(temp, id) == 0) {
      snprintf(out, outSize, "%s", masterNames[i].name);
      return;
    }
  }
  snprintf(out, outSize, "Unknown Fan");
}

int findDevice(const char* id) {
  // 1st pass – check exact match
  for (int i = 0; i < deviceCount; i++)
    if (strcmp(id, devices[i].deviceId) == 0)
      return i;

  // 2nd pass – check id + _R1 match
  char temp[MAX_ID_LEN];
  for (int i = 0; i < deviceCount; i++) {
    snprintf(temp, sizeof(temp), "%s_R1", devices[i].deviceId);
    if (strcmp(id, temp) == 0)
      return i;
  }

  return -1;
}

/***************************** ALEXA SEND ******************************/
void sendSensorValueToAlexa(const char* name, const char* reading) {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WARN] Cannot push to Alexa — WiFi Down");
    return;
  }

  // ---- FIX: encode spaces (" " → "%20") in name ----
  char encodedName[128];
  int j = 0;
  for (int i = 0; name[i] && j < sizeof(encodedName) - 4; i++) {
    if (name[i] == ' ') {
      encodedName[j++] = '%';
      encodedName[j++] = '2';
      encodedName[j++] = '0';
    } else {
      encodedName[j++] = name[i];
    }
  }
  encodedName[j] = '\0';

  // ---- FIX: encode spaces (" " → "%20") in reading ----
  char encodedReading[128];
  int k = 0;
  for (int i = 0; reading[i] && k < sizeof(encodedReading) - 4; i++) {
    if (reading[i] == ' ') {
      encodedReading[k++] = '%';
      encodedReading[k++] = '2';
      encodedReading[k++] = '0';
    } else {
      encodedReading[k++] = reading[i];
    }
  }
  encodedReading[k] = '\0';

  char url[256];
  snprintf(url, sizeof(url),
           "https://home-automation.vadrin.com/droid/%s/upsert/intent/%s/reading/%s",
           DROID_ID, encodedName, encodedReading);  // ← only change!

  Serial.printf("[HTTP] → Sending to Alexa: %s\n", url);

  client.setInsecure();
  https.setTimeout(5000);
  if (!https.begin(client, url)) {
    Serial.println("[HTTP] begin() failed — skipping");
    return;
  }
  int code = https.GET();

  Serial.printf("[HTTP] Response Code: %d\n", code);

  https.end();
  client.stop();
}

/****************************** UPSERT DEVICE ***************************/
void upsertDevice(const char* id, bool power, bool led, int speed,
                  bool pushToAlexa) {

  if (!id) return;

  int i = findDevice(id);
  bool isNew = (i < 0);

  if (isNew) {
    if (deviceCount >= MAX_DEVICES) {
      Serial.printf("[DEVICE] Table full (%d) — ignoring %s\n", MAX_DEVICES, id);
      return;
    }
    i = deviceCount++;
    snprintf(devices[i].deviceId, sizeof(devices[i].deviceId), "%s", id);
  }

  devices[i].power = power;
  devices[i].switchOn = true;
  devices[i].led = led;
  devices[i].speed = speed;
  devices[i].lastHeartbeat = millis();

  Serial.printf("[DEVICE] %s %s → Power:%d LED:%d Speed:%d\n",
                isNew ? "NEW" : "UPDATE", id, power, led, speed);

  if (!pushToAlexa) return;

  char msg[48];
  if (power && speed > 0) snprintf(msg, sizeof(msg), "On. Speed %d.", speed);
  else if (!power && speed > 0) snprintf(msg, sizeof(msg), "On. Standby.");
  else snprintf(msg, sizeof(msg), "On");

  char fanName[32];
  getDeviceName(id, fanName, sizeof(fanName));

  sendSensorValueToAlexa(fanName, msg);
}

/**************************** PROCESS FAN EVENT *************************/
void processFanEvent(const char* hex) {

  int hexLen = (int)strlen(hex);
  if (hexLen < 2 || hexLen > 1024) return;

  char json[512] = { 0 };
  for (int i = 0, j = 0; i < hexLen && j < 511; i += 2, j++) {
    char h[3] = { hex[i], hex[i + 1], 0 };
    json[j] = (char)strtol(h, NULL, 16);
  }

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, json)) {
    Serial.println("[ERR] JSON Parse Failed");
    return;
  }

  const char* id = doc["device_id"];
  const char* state = doc["state_string"];
  if (!id || !state) {
    Serial.println("[ERR] Missing device_id or state_string in JSON");
    return;
  }

  const char* comma = strchr(state, ',');
  int commaPos = comma ? (int)(comma - state) : (int)strlen(state);
  char numBuf[16] = { 0 };
  if (commaPos > 0 && commaPos < (int)sizeof(numBuf))
    snprintf(numBuf, sizeof(numBuf), "%.*s", commaPos, state);
  uint32_t encoded = strtoul(numBuf, NULL, 10);

  bool power = (encoded & 0x10) > 0;
  bool led = (encoded & 0x20) > 0;
  int speed = (encoded & 0x07);

  upsertDevice(id, power, led, speed, true);
}

void loadDefaultDevices() {
  int masterCount = sizeof(masterNames) / sizeof(masterNames[0]);
  for (int i = 0; i < masterCount; i++) {
    upsertDevice(masterNames[i].id, false, false, 0, false);
    yield();
  }
  Serial.printf("Loaded %d default devices into memory.\n", masterCount);
  Serial.printf("[BOOT] Heap after defaults: %u bytes\n", ESP.getFreeHeap());
}

// ---------------------------------------------------------------------------
// Upsert helpers
// ---------------------------------------------------------------------------
void upsertDevice_Off() {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].switchOn && (millis() - devices[i].lastHeartbeat > 10000)) {
      // Looks like switch is recently turned off
      devices[i].switchOn = false;
      char msg[48];
      snprintf(msg, sizeof(msg), "Off");
      char fanName[32];
      getDeviceName(devices[i].deviceId, fanName, sizeof(fanName));
      Serial.println("[DEVICE] switch Off");
      sendSensorValueToAlexa(fanName, msg);
    }
  }
}

/****************************** SETUP **********************************/
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\n========== BOOT ==========");
#if defined(ESP8266)
  Serial.printf("[BOOT] Reset reason : %s\n", ESP.getResetReason().c_str());
  Serial.printf("[BOOT] Reset info   : %s\n", ESP.getResetInfo().c_str());
#endif
  Serial.printf("[BOOT] Free heap    : %u bytes\n", ESP.getFreeHeap());

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
#if defined(ESP32)
  WiFi.setSleep(false);
#endif
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Heap after WiFi: %u bytes\n", ESP.getFreeHeap());

  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on %d\n", UDP_PORT);

  Serial.println("Loading default devices into memory...");
  loadDefaultDevices();

  wifiLastConnected = millis();
  Serial.println("Setup complete.");
}

/******************************* LOOP **********************************/
uint32_t logTimer = 0;
uint32_t lastOffCheck = 0;
const uint32_t OFF_CHECK_INTERVAL = 2000;  // check heartbeat timeouts every 2s

void loop() {

  wifiWatchdog();
  yield();

  if (millis() - logTimer > HEAP_PRINT_TIME) {
    logTimer = millis();
    Serial.printf("[HEAP] Free RAM: %u bytes\n", ESP.getFreeHeap());
  }

  int size = udp.parsePacket();
  if (size > 0) {

    static char buf[2048];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = 0;

    //Serial.printf("[UDP] Packet Received (%d bytes) → %s\n", len, buf);

    bool isHex = true;
    for (int i = 0; i < len; i++)
      if (!isxdigit(buf[i])) isHex = false;

    if (isHex) {
      //Serial.println("[TYPE] → FAN EVENT");
      processFanEvent(buf);
    } else {
      int idx = findDevice(buf);
      if (idx >= 0) {
        devices[idx].lastHeartbeat = millis();
        if (!devices[idx].switchOn) {
          upsertDevice(buf, false, false, 0, true);
        }
      } else if (len > 0 && len < MAX_ID_LEN) {
        upsertDevice(buf, false, false, 0, true);
      }
    }
  }

  // ---- Turn off DEVICES with no heartbeat (throttled) ----
  if (millis() - lastOffCheck > OFF_CHECK_INTERVAL) {
    lastOffCheck = millis();
    upsertDevice_Off();
  }

  yield();
}