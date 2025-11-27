/**************** MEMORY OPTIMIZED + DEBUG LOGGING ENABLED *******************/
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#define MAX_DEVICES 20
#define MAX_ID_LEN 20
#define MAX_NAME_LEN 32
#define UDP_PORT 5625

const char* ssid = "XXXX";
const char* password = "YYYY";
const char* DROID_ID = "ABCD";

WiFiUDP udp;
static WiFiClientSecure client;
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
uint32_t wifiLastConnected = millis();
const uint32_t WIFI_TIMEOUT_REBOOT = 60000;      // 60 sec offline → reboot
const uint32_t WIFI_RETRY_INTERVAL = 5000;       // retry every 5 sec

void wifiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiLastConnected = millis();
    return;
  }

  // WiFi Lost → check how long
  if (millis() - wifiLastConnected > WIFI_RETRY_INTERVAL) {
    Serial.println("[WIFI] Lost. Attempting reconnect...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    wifiLastConnected = millis();  // reset timer after retry
  }

  // Still down too long? → auto restart ESP
  if (millis() - wifiLastConnected > WIFI_TIMEOUT_REBOOT) {
    Serial.println("\n[FAIL] WiFi offline too long → Restarting ESP32...");
    delay(500);
    ESP.restart();
  }
}

/******************************** UTIL *********************************/
void getDeviceName(const char* id, char* out) {
  for (int i = 0; i < sizeof(masterNames) / sizeof(masterNames[0]); i++) {
    if (strcmp(masterNames[i].id, id) == 0) {
      strcpy(out, masterNames[i].name);
      return;
    }
  }
  char temp[MAX_ID_LEN];
  for (int i = 0; i < sizeof(masterNames) / sizeof(masterNames[0]); i++) {
    snprintf(temp, sizeof(temp), "%s_R1", masterNames[i].id);
    if (strcmp(temp, id) == 0) {
      strcpy(out, masterNames[i].name);
      return;
    }
  }
  strcpy(out, "Unknown Fan");
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
  https.begin(client, url);
  int code = https.GET();

  Serial.printf("[HTTP] Response Code: %d\n", code);

  https.end();
}

/****************************** UPSERT DEVICE ***************************/
void upsertDevice(const char* id, bool power, bool led, int speed) {

  int i = findDevice(id);
  bool isNew = (i < 0);

  if (isNew && deviceCount < MAX_DEVICES) {
    i = deviceCount++;
    strcpy(devices[i].deviceId, id);
  }

  devices[i].power = power;
  devices[i].switchOn = true;
  devices[i].led = led;
  devices[i].speed = speed;
  devices[i].lastHeartbeat = millis();

  Serial.printf("[DEVICE] %s %s → Power:%d LED:%d Speed:%d\n",
                isNew ? "NEW" : "UPDATE", id, power, led, speed);

  char msg[48];
  if (!devices[i].switchOn) strcpy(msg, "Off");
  else if (power && speed > 0) snprintf(msg, sizeof(msg), "On. Speed %d.", speed);
  else if (!power && speed > 0) strcpy(msg, "On. Standby.");
  else strcpy(msg, "On");

  char fanName[32];
  getDeviceName(id, fanName);

  sendSensorValueToAlexa(fanName, msg);
}

/**************************** PROCESS FAN EVENT *************************/
void processFanEvent(const char* hex) {
  Serial.printf("[UDP] HEX Received (%d bytes) -> Decoding JSON...\n", strlen(hex));

  char json[512] = { 0 };
  for (int i = 0, j = 0; i < (int)strlen(hex) && j < 511; i += 2, j++) {
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
  int comma = strchr(state, ',') ? strchr(state, ',') - state : strlen(state);
  uint32_t encoded = strtoul(String(state).substring(0, comma).c_str(), NULL, 10);

  bool power = (encoded & 0x10) > 0;
  bool led = (encoded & 0x20) > 0;
  int speed = (encoded & 0x07);

  upsertDevice(id, power, led, speed);
}

void loadDefaultDevices() {
  int masterCount = sizeof(masterNames) / sizeof(masterNames[0]);
  for (int i = 0; i < masterCount; i++) {
    upsertDevice(masterNames[i].id, false, false, 0);
  }
  Serial.println("Default devices loaded from masterNames.");
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
      strcpy(msg, "Off");
      char fanName[32];
      getDeviceName(devices[i].deviceId, fanName);
      sendSensorValueToAlexa(fanName, msg);
    }
  }
}

/****************************** SETUP **********************************/
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on %d\n", UDP_PORT);

  Serial.println("Loading default devices...");
  loadDefaultDevices();
}

/******************************* LOOP **********************************/
uint32_t logTimer = 0;

void loop() {

  wifiWatchdog();

  /* Print heap every 10s */
  if (millis() - logTimer > 10000) {
    logTimer = millis();
    Serial.printf("[HEAP] Free RAM: %u bytes\n", ESP.getFreeHeap());
  }

  int size = udp.parsePacket();
  if (size > 0) {

    static char buf[2048];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = 0;

    Serial.printf("[UDP] Packet Received (%d bytes) → %s\n", len, buf);

    bool isHex = true;
    for (int i = 0; i < len; i++)
      if (!isxdigit(buf[i])) isHex = false;

    if (isHex) {
      Serial.println("[TYPE] → FAN EVENT");
      processFanEvent(buf);
    } else {
      Serial.println("[TYPE] → HEARTBEAT");
      int idx = findDevice(buf);
      if(idx >= 0)
        devices[idx].lastHeartbeat = millis();
      if (idx < 0 || devices[idx].switchOn == false) {
        upsertDevice(buf, false, false, 0);
      }
    }
  }

  // ---- Turn off DEVICES with no heartbeat----
  upsertDevice_Off();
}