#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

const char* ssid = "ACT-ai_101743562903";
const char* password = "22469425";
const String DROID_ID = "R2D2";

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

void sendSensorValueToAlexa(String name, String reading) {
  String encodedName = "";
  for (int i = 0; i < name.length(); i++) encodedName += (name.charAt(i) == ' ') ? "%20" : String(name.charAt(i));
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String fullUrl = "https://home-automation.vadrin.com/droid/" + DROID_ID + "/upsert/intent/" + encodedName + "/reading/" + reading;
  Serial.println("Requesting " + fullUrl);
  if (https.begin(client, fullUrl)) {
    int httpCode = https.GET();
    Serial.println("============== Response code: " + String(httpCode));
    if (httpCode > 0) {
      Serial.println(https.getString());
    }
    https.end();
  } else {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
}

// Add or update device
void upsertDeviceInfo(String devId, bool power, bool led, int speed) {
  int index = findDeviceIndex(devId);

  if (index == -1) {
    // NEW device
    if (deviceCount < 20) {
      devices[deviceCount].deviceId = devId;
      devices[deviceCount].power = power;
      devices[deviceCount].led = led;
      devices[deviceCount].speed = speed;
      devices[deviceCount].lastHeartbeatTime = millis();
      deviceCount++;
    }
  } else {
    // UPDATE existing device
    devices[index].power = power;
    devices[index].led = led;
    devices[index].speed = speed;
    devices[index].lastHeartbeatTime = millis();
  }

  sendDeviceDataToAlexa(devId);
}

void sendDeviceDataToAlexa(String devId){
  int currentDeviceIndex = findDeviceIndex(devId);
  String constructDeviceMessage;

  if (devices[currentDeviceIndex].power) {
    constructDeviceMessage = "On";
    if (devices[currentDeviceIndex].speed == 0) {
      //constructDeviceMessage += ".%20Standby.";
    }else {
      constructDeviceMessage += ".%20Speed%20" + String(devices[currentDeviceIndex].speed) + ".";
    }
  } else {
    constructDeviceMessage = "Off";
  }

  //Serial.println(constructDeviceMessage);
  sendSensorValueToAlexa(getDeviceName(devId), constructDeviceMessage);
}

void trackHeartbeatTime(String devId) {
  int index = findDeviceIndex(devId);
  if (index == -1) {
    // NEW device. Some1 has to add the device before I begin to track its heartbeat
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
// WiFi
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());

  udp.begin(UDP_PORT);
  Serial.println("Listening on UDP port 5625...");
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

    Serial.print("  Power: ");
    Serial.println(devices[i].power);

    Serial.print("  LED: ");
    Serial.println(devices[i].led);

    Serial.print("  Speed: ");
    Serial.println(devices[i].speed);

    Serial.println("-------------------------");
  }
}


void turnOffHeartlessDevices() {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].power && (millis() - devices[i].lastHeartbeatTime > 10000)) {
      upsertDeviceInfo(devices[i].deviceId, false, false, 0);
    }
  }
}

String getDeviceIdFromHeartbeat(const String& input) {
  int pos = input.indexOf('_');
  if (pos == -1) {
    // No '-' found, print full string
    return input;
  } else {
    return input.substring(0, pos);
  }
}

// ---------------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------------
void loop() {

  // ---- READ UDP PACKETS ----
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incoming[2048];
    int len = udp.read(incoming, sizeof(incoming) - 1);
    if (len > 0) incoming[len] = 0;
    String hexString = String(incoming);
    // check if its heardbeat message or state transition message
    if (!isHexString(hexString)) {
      // check if it is a device heart beat
      if (hexString.indexOf('_') != -1) {
        String devId = getDeviceIdFromHeartbeat(hexString);
        // Auto discover- check if this device is not yet added to device collection
        if (findDeviceIndex(devId) == -1) {
          //since its not there create a record
          upsertDeviceInfo(devId, true, false, 0);
        } else {
          //since its already discovered just track its heartbeat
          trackHeartbeatTime(devId);
        }
      }
      return;
    }
    // decode HEX → ASCII JSON
    String jsonText = hexToAscii(hexString);
    DynamicJsonDocument doc(1024);
    if (deserializeJson(doc, jsonText)) return;
    String device_id = doc["device_id"];
    String message_id = doc["message_id"];
    String state_string = doc["state_string"];

    //Skip-2. Sometimes when we keep the mobile app open these messages spam us
    if (message_id == "internet_query") return;

    // Extract encoded first field
    int firstComma = state_string.indexOf(",");
    String firstField = state_string.substring(0, firstComma);
    uint32_t encodedValue = (uint32_t)strtoul(firstField.c_str(), NULL, 10);
    bool power = (encodedValue & 0x10) > 0;
    bool led = (encodedValue & 0x20) > 0;
    int speed = (encodedValue & 0x07);
    upsertDeviceInfo(device_id, power, led, speed);
  }

  // ---- Remove DEVICES with no heartbeat ----
  turnOffHeartlessDevices();
  
  // ---- PRINT DEVICES EVERY 5 SECONDS ----
  if (millis() - lastPrintTime >= 5000) {
    lastPrintTime = millis();
    printAllDevices();
  }

}