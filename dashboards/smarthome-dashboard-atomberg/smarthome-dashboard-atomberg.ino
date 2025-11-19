#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

const char* ssid = "XXXX";
const char* password = "YYYY";
const String DROID_ID = "ABCD";

WiFiUDP udp;
const unsigned int UDP_PORT = 5625;

// ---------------------------------------------------------------------------
// MASTER DEVICE NAME TABLE (device_id → device_name)
// ---------------------------------------------------------------------------
struct MasterDeviceName {
  String deviceId;
  String deviceName;
};

// Add your known devices here
MasterDeviceName masterNames[] = {
  { "24587c849608", "Master Fan" },
  { "3c8427853c00", "Kids Fan" },
  { "3c84278a9f84", "Parents Fan" },
  { "80659934c6b8", "Living Fan" },
  { "3c842783d658", "Dining Fan" },
};

const int masterCount = sizeof(masterNames) / sizeof(masterNames[0]);

// Lookup function
String getDeviceName(const String& devId) {
  for (int i = 0; i < masterCount; i++) {
    if (masterNames[i].deviceId == devId) {
      return masterNames[i].deviceName;
    }
  }
  return "Unknown Fan";
}

// ---------------------------------------------------------------------------
// DEVICE COLLECTION
// ---------------------------------------------------------------------------
struct DeviceInfo {
  String deviceId;
  bool power;
  bool switchOn;
  bool led;
  int speed;
  unsigned long lastHeartbeatTime;
};

DeviceInfo devices[20];  // max 20 devices
int deviceCount = 0;

// Find index of device in array
int findDeviceIndex(const String& devId) {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].deviceId == devId)
      return i;
  }
  return -1;
}

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void sendDeviceDataToAlexa(String devId);

// ---------------------------------------------------------------------------
// Safe URL encoding for names (spaces -> %20)
// ---------------------------------------------------------------------------
String urlEncodeSpaces(const String& name) {
  String encoded = "";
  for (int i = 0; i < name.length(); i++) {
    char c = name.charAt(i);
    if (c == ' ') encoded += "%20";
    else encoded += c;
  }
  return encoded;
}

// ---------------------------------------------------------------------------
// sendSensorValueToAlexa (ONLY called when WiFi is connected)
// ---------------------------------------------------------------------------
void sendSensorValueToAlexa(String name, String reading) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[sendSensorValueToAlexa] WiFi not connected - skipping send");
    return;
  }

  String encodedName = urlEncodeSpaces(name);
  WiFiClientSecure client;
  client.setInsecure();  // accept all certs - keep if you already used it
  client.setTimeout(30);  // small read timeout (seconds)

  HTTPClient https;
  String fullUrl = "https://home-automation.vadrin.com/droid/" + DROID_ID + "/upsert/intent/" + encodedName + "/reading/" + reading;
  Serial.println("[sendSensorValueToAlexa] Requesting " + fullUrl);

  // Begin connection
  if (!https.begin(client, fullUrl)) {
    Serial.println("[sendSensorValueToAlexa] HTTPS begin() failed");
    Serial.println("[sendSensorValueToAlexa] HTTPS begin() failed — restarting ESP32");
    delay(500);
    ESP.restart();
  }

  // Make GET (this call is blocking but we guard it by skipping if WiFi down and using short read timeout)
  int httpCode = https.GET();
  Serial.println("[sendSensorValueToAlexa] Response code: " + String(httpCode));
  if (httpCode > 0) {
    String payload = https.getString();
    Serial.println(payload);
  } else {
    Serial.println("[sendSensorValueToAlexa] HTTPS GET failed or timed out — restarting ESP32");
    delay(500);
    ESP.restart();
  }

  https.end();
}

// ---------------------------------------------------------------------------
// Upsert helpers
// ---------------------------------------------------------------------------
void upsertDevice_Switch() {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].switchOn && (millis() - devices[i].lastHeartbeatTime > 10000)) {
      // Looks like switch is just turned off
      devices[i].switchOn = false;
      sendDeviceDataToAlexa(devices[i].deviceId);
    }
  }
}

// Add or update device
void upsertDevice_Info(String devId, bool power, bool led, int speed) {
  int index = findDeviceIndex(devId);

  if (index == -1) {
    // NEW device
    if (deviceCount < 20) {
      devices[deviceCount].deviceId = devId;
      devices[deviceCount].power = power;
      devices[deviceCount].switchOn = true;
      devices[deviceCount].led = led;
      devices[deviceCount].speed = speed;
      devices[deviceCount].lastHeartbeatTime = millis();
      deviceCount++;
    }
  } else {
    // UPDATE existing device
    devices[index].power = power;
    devices[index].switchOn = true;
    devices[index].led = led;
    devices[index].speed = speed;
    devices[index].lastHeartbeatTime = millis();
  }

  sendDeviceDataToAlexa(devId);
}

void sendDeviceDataToAlexa(String devId) {
  int currentDeviceIndex = findDeviceIndex(devId);
  if (currentDeviceIndex < 0) return;

  String constructDeviceMessage;
  if (devices[currentDeviceIndex].switchOn) {
    if (devices[currentDeviceIndex].power && devices[currentDeviceIndex].speed != 0) {
      constructDeviceMessage = "On.%20Speed%20" + String(devices[currentDeviceIndex].speed) + ".";
    } else if (!devices[currentDeviceIndex].power && devices[currentDeviceIndex].speed != 0) {
      constructDeviceMessage = "On.%20Standby.";
    } else {
      constructDeviceMessage = "On";
    }
  } else {
    constructDeviceMessage = "Off";
  }

  // Only attempt network call when WiFi connected — sendSensorValueToAlexa also double-checks
  if (WiFi.status() == WL_CONNECTED) {
    sendSensorValueToAlexa(getDeviceName(devId), constructDeviceMessage);
  } else {
    Serial.println("[sendDeviceDataToAlexa] WiFi down. Skipping Alexa update for " + devId);
  }
}

void trackHeartbeatTime(String devId) {
  int index = findDeviceIndex(devId);
  if (index == -1) {
    // NEW device. No need to track its heartbeat
  } else {
    // UPDATE existing device
    devices[index].lastHeartbeatTime = millis();
  }
}

// ---------------------------------------------------------------------------
// Convert HEX → ASCII
// ---------------------------------------------------------------------------
String hexToAscii(const String& hex) {
  String ascii = "";
  for (int i = 0; i < hex.length(); i += 2) {
    String part = hex.substring(i, i + 2);
    char ch = (char)strtol(part.c_str(), NULL, 16);
    ascii += ch;
  }
  return ascii;
}

// ---------------------------------------------------------------------------
// WiFi recovery/watcher
// ---------------------------------------------------------------------------
unsigned long lastWiFiCheck = 0;
bool udpStarted = false;

void ensureUDPStarted() {
  if (!udpStarted) {
    if (udp.begin(UDP_PORT)) {
      udpStarted = true;
      Serial.println("[ensureUDPStarted] UDP begun on port " + String(UDP_PORT));
    } else {
      Serial.println("[ensureUDPStarted] UDP begin failed");
    }
  }
}

void stopUDP() {
  // There's no direct udp.stop(); but we can set flag so begin will be called again on reconnect
  udpStarted = false;
  // On some platforms you can call udp.stop(); but leaving it to re-begin is acceptable.
  Serial.println("[stopUDP] Marking UDP as stopped; will re-init after reconnect");
}

// Called regularly from loop, lightweight
void ensureWiFiConnected() {
  // check only periodically
  if (millis() - lastWiFiCheck < 5000) return;
  lastWiFiCheck = millis();

  wl_status_t s = WiFi.status();
  if (s == WL_CONNECTED) {
    // Ensure UDP started after a reconnect or initial connect
    ensureUDPStarted();
    return;
  }

  Serial.println("[ensureWiFiConnected] WiFi not connected. Attempting reconnect.");

  // Try a short reconnect attempt (non-blocking overall)
  WiFi.disconnect(true);  // clear previous settings and reconnect fresh
  delay(200);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  // wait up to 6 seconds but without blocking too long
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 6000) {
    delay(200);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[ensureWiFiConnected] Reconnected to WiFi. IP: " + WiFi.localIP().toString());
    // re-init UDP if needed
    ensureUDPStarted();
  } else {
    Serial.println("\n[ensureWiFiConnected] Reconnect attempt failed. Will retry later.");
  }
}

// Optionally, detect WiFi events and react immediately
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("[WiFiEvent] GOT_IP: " + WiFi.localIP().toString());
      ensureUDPStarted();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("[WiFiEvent] DISCONNECTED");
      stopUDP();
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// HEX Validator
// ---------------------------------------------------------------------------
bool isHexString(const String& hexString) {
  if (hexString.length() == 0) return false;

  for (int i = 0; i < hexString.length(); i++) {
    char c = hexString.charAt(i);
    bool isDigit = (c >= '0' && c <= '9');
    bool isUpperHex = (c >= 'A' && c <= 'F');
    bool isLowerHex = (c >= 'a' && c <= 'f');
    if (!isDigit && !isUpperHex && !isLowerHex) return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// PRINT DEVICE COLLECTION EVERY 5 SECONDS
// ---------------------------------------------------------------------------
unsigned long lastPrintTime = 0;

void printAllDevices() {
  Serial.println("\n===== DEVICE TABLE =====");
  for (int i = 0; i < deviceCount; i++) {
    Serial.print("Device: ");
    Serial.println(devices[i].deviceId);

    Serial.print("  Name: ");
    Serial.println(getDeviceName(devices[i].deviceId));

    Serial.print("  switchOn: ");
    Serial.println(devices[i].switchOn);

    Serial.print("  Power: ");
    Serial.println(devices[i].power);

    Serial.print("  LED: ");
    Serial.println(devices[i].led);

    Serial.print("  Speed: ");
    Serial.println(devices[i].speed);

    Serial.println("-------------------------");
  }
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
}

String getDeviceIdFromHeartbeat(const String& input) {
  int pos = input.indexOf('_');
  if (pos == -1) {
    // No '_' found, return full string
    return input;
  } else {
    return input.substring(0, pos);
  }
}

void processHeartbeat(String incomingString) {
  if (incomingString.indexOf('_') != -1) {
    String devId = getDeviceIdFromHeartbeat(incomingString);
    // Since its not there in collection create a record and register its as switchOn
    // Also, if its already created but tracked as switchOff, then track it as switchOn
    if (findDeviceIndex(devId) == -1 || !devices[findDeviceIndex(devId)].switchOn) {
      upsertDevice_Info(devId, false, false, 0);
    }
    trackHeartbeatTime(devId);
  }
}

void processFanEvent(String incomingString) {
  // decode HEX → ASCII JSON
  String jsonText = hexToAscii(incomingString);
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, jsonText);
  if (err) {
    Serial.println("[processFanEvent] JSON parse failed");
    return;
  }

  String device_id = doc["device_id"] | "";
  String message_id = doc["message_id"] | "";
  String state_string = doc["state_string"] | "";

  // Skip-2. Sometimes when we keep the mobile app open these messages spam us
  if (message_id == "internet_query") return;

  // Extract encoded first field
  int firstComma = state_string.indexOf(",");
  String firstField = (firstComma == -1) ? state_string : state_string.substring(0, firstComma);
  uint32_t encodedValue = (uint32_t)strtoul(firstField.c_str(), NULL, 10);
  bool power = (encodedValue & 0x10) > 0;
  bool led = (encodedValue & 0x20) > 0;
  int speed = (encodedValue & 0x07);
  upsertDevice_Info(device_id, power, led, speed);
}

// ---------------------------------------------------------------------------
// LOAD DEFAULT DEVICES FROM masterNames[]
// power = false, led = false, speed = 0
// ---------------------------------------------------------------------------
void loadDefaultDevices() {
  for (int i = 0; i < masterCount; i++) {
    upsertDevice_Info(masterNames[i].deviceId, false, false, 0);
  }
  Serial.println("Default devices loaded from masterNames.");
}

// ---------------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Connecting to WiFi...");
  WiFi.onEvent(onWiFiEvent);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(200);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.println("IP Address: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nInitial WiFi connect failed - will continue and try reconnects in loop.");
  }

  // start UDP if WiFi is already connected; otherwise the ensureWiFiConnected will start it
  if (WiFi.status() == WL_CONNECTED) {
    ensureUDPStarted();
  } else {
    udpStarted = false;
  }

  // Load master device defaults
  loadDefaultDevices();
}

void loop() {
  // ---- WiFi watchdog (non blocking overall) ----
  ensureWiFiConnected();

  // ---- READ UDP PACKETS (only if UDP is started)----
  if (udpStarted) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      // Keep buffer on stack; ensure also we don't overflow
      static char incoming[2048];
      int len = udp.read(incoming, sizeof(incoming) - 1);
      if (len > 0) incoming[len] = 0;
      else incoming[0] = 0;

      String incomingString = String(incoming);
      // Decide if this is a HEX (fan event) or plain heartbeat
      if (isHexString(incomingString)) {
        // Fan event from device
        processFanEvent(incomingString);
      } else {
        // Heartbeat / other
        processHeartbeat(incomingString);
      }
    }
  }

  // ---- Turn off DEVICES with no heartbeat----
  upsertDevice_Switch();

  // ---- PRINT DEVICES EVERY 5 SECONDS ----
  if (millis() - lastPrintTime >= 5000) {
    lastPrintTime = millis();
    printAllDevices();
  }

  // Yield to other tasks and keep things responsive
  delay(10);
}