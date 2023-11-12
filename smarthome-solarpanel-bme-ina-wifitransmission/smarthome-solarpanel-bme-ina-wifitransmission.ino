#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>  //you can download the library from https://github.com/gv-prashanth/arduino-projects/tree/master/libraries
#endif
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_INA219.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <TimeLib.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define WIFI_SSID "GTS"
#define WIFI_PASS "0607252609"

const String DROID_ID = "C3PO";
float CUTOFF_VOLTAGE = 11.5;            //volts
float BATTERY_RATED_VOLTAGE = 12.4;     //volts
float BATTERY_RATED_CAPACITY = 7000.0;  //mah
float CUTOFF_CURRENT = 3000;            //milliAmps
float MIN_LOAD_CURRENT = 50;            //ma
int OUTPUT_PIN = 1;                     //pin 1
unsigned long COOLDOWN_TIME = 3600000;  //milliSeconds
float PRECISSION_POWER_A = 3.0;         //w
float PRECISSION_VOLTAGE_A = 1.0;       //v
float PRECISSION_POWER_B = 1.0;         //w
float PRECISSION_VOLTAGE_B = 0.1;       //v
float PRECISSION_TEMP = 1.0;            //degrees
float PRECISSION_HUMID = 2.0;           //percentage
int timeToStartLoad = 8;
int timeToEndLoad = 11;

//Dont touch below
Adafruit_BME280 bme;
Adafruit_INA219 ina219_A;        //ina connected to solar panel output
Adafruit_INA219 ina219_B(0x41);  //ina connected to battery output
ADC_MODE(ADC_VCC);
const String WORLD_TIME_API = "http://worldtimeapi.org/api/ip";  // Fetch the time from World Time API

//Ina readings
float A_shuntvoltage, A_busvoltage, A_current_mA, A_power_W, A_loadvoltage, B_shuntvoltage, B_busvoltage, B_current_mA, B_power_W, B_loadvoltage;
float prev_A_power_W, prev_A_loadvoltage, prev_B_power_W, prev_B_loadvoltage;
boolean INAChangeDetected;

//BME readings
float bme_readTemperature, bme_readPressure, bme_readHumidity, bme_readAltitude;
float prev_bme_readTemperature, prev_bme_readHumidity;
boolean BMEChangeDetected;

//Danger
boolean prevLoopBatterInDanger;
unsigned long turnOnBatteryAccessAt;  //milliSeconds

String error = "";

void setup() {
  //IF YOU WANT TO ENABLE SERIAL MONITOR THEN DONT USE GPIO 1
  //Serial.begin(74880);
  //while (!Serial)
  //  ;
  pinMode(OUTPUT_PIN, OUTPUT);

  // Wi-Fi connection
  wifiSetup();

  Serial.println(F("BME Initializing..."));
  Wire.begin(0, 2);  // SDA, SCL
  boolean bmestatus = bme.begin(0x76);
  if (!bmestatus) {
    error += "bme ";
  }
  Serial.println(F("INA219_A Initializing..."));
  if (!ina219_A.begin()) {
    Serial.println("Failed to find INA219_A chip");
    error += "ina219_A ";
  }
  Serial.println(F("INA219_B Initializing..."));
  if (!ina219_B.begin()) {
    Serial.println("Failed to find INA219_B chip");
    error += "ina219_B ";
  }
  INAChangeDetected = true;
  BMEChangeDetected = true;
  prevLoopBatterInDanger = false;
  Serial.println("Errors: " + error);

  //Lets fetch and parse once to be ready to display immediatly.
  while (timeStatus() != timeSet) {
    Serial.println("trying to fetch time");
    fetchAndLoadCurrentTimeFromWeb();
  }
}

void loop() {
  loadINAReadings();
  TakeActionBasedOnBatteryStatus();
  checkAndsendToAlexaINAReadings();
  loadBMEReadings();
  checkAndsendToAlexaBMEReadings();
  //printESPReadings();
}

void TakeActionBasedOnBatteryStatus() {
  unsigned long currentTime = millis();
  if (isBatterCurrentlyInDanger() && !prevLoopBatterInDanger) {
    //Battery Just got into Danger.
    Serial.println("Battery just got into danger zone.");
    turnOnBatteryAccessAt = currentTime + COOLDOWN_TIME;
    INAChangeDetected = true;
  }
  if (!isBatterCurrentlyInDanger() && prevLoopBatterInDanger) {
    //Battery Just got out of Danger.
    INAChangeDetected = true;
  }
  if (isBatterCurrentlyInDanger())
    digitalWrite(OUTPUT_PIN, LOW);
  else if (isBatteryOutOfDanagerButInCoolOff())
    digitalWrite(OUTPUT_PIN, LOW);
  else
    digitalWrite(OUTPUT_PIN, HIGH);
  prevLoopBatterInDanger = isBatterCurrentlyInDanger();
}

boolean isBatteryOutOfDanagerButInCoolOff() {
  unsigned long currentTime = millis();
  return (!isBatterCurrentlyInDanger() && (currentTime < turnOnBatteryAccessAt));
}

boolean isBatterCurrentlyInDanger() {
  return ((B_loadvoltage < CUTOFF_VOLTAGE) || (B_current_mA > CUTOFF_CURRENT) || hour() < timeToStartLoad || hour() > timeToEndLoad);
}

void printESPReadings() {
  //esp voltage
  float espVoltage = ESP.getVcc() / 1000.00;
  Serial.print("ESP Voltage = ");
  Serial.print(espVoltage);
  Serial.println(" V");
}

void loadINAReadings() {
  //Ina_A readings
  A_shuntvoltage = ina219_A.getShuntVoltage_mV();
  A_busvoltage = ina219_A.getBusVoltage_V();
  A_current_mA = ina219_A.getCurrent_mA();
  A_power_W = ina219_A.getPower_mW() / 1000;
  A_loadvoltage = A_busvoltage + (A_shuntvoltage / 1000);
  //Ina_B readings
  B_shuntvoltage = ina219_B.getShuntVoltage_mV();
  B_busvoltage = ina219_B.getBusVoltage_V();
  B_current_mA = ina219_B.getCurrent_mA();
  B_power_W = ina219_B.getPower_mW() / 1000;
  B_loadvoltage = B_busvoltage + (B_shuntvoltage / 1000);
  if (abs(A_power_W - prev_A_power_W) > PRECISSION_POWER_A || abs(A_loadvoltage - prev_A_loadvoltage) > PRECISSION_VOLTAGE_A) {
    INAChangeDetected = true;
    prev_A_power_W = A_power_W;
    prev_A_loadvoltage = A_loadvoltage;
  }
  if (abs(B_power_W - prev_B_power_W) > PRECISSION_POWER_B || abs(B_loadvoltage - prev_B_loadvoltage) > PRECISSION_VOLTAGE_B) {
    INAChangeDetected = true;
    prev_B_power_W = B_power_W;
    prev_B_loadvoltage = B_loadvoltage;
  }
}

int calculateBatteryPercentage() {
  int batteryPercentage = ((100.0 - 0.0) / (BATTERY_RATED_VOLTAGE - CUTOFF_VOLTAGE)) * B_loadvoltage + (0.0 - ((100.0 - 0.0) / (BATTERY_RATED_VOLTAGE - CUTOFF_VOLTAGE)) * CUTOFF_VOLTAGE);
  if (batteryPercentage >= 100)
    batteryPercentage = 100;
  return batteryPercentage;
}

boolean isBatteryDraining() {
  return B_current_mA > A_current_mA;
}

String timeForFullDrainInHours() {
  if (B_current_mA == A_current_mA)
    return String("a%20day");
  float availableMAH = (((float)calculateBatteryPercentage()) / 100.0) * BATTERY_RATED_CAPACITY;
  float hoursToDrain = availableMAH / (B_current_mA - A_current_mA);
  if (hoursToDrain > 23 || hoursToDrain < 0)
    return String("a%20day");
  if ((int)hoursToDrain == 0)
    return String("soon");
  return String((int)hoursToDrain) + "%20hrs";
}

String timeForFullChargeInHours() {
  if (B_current_mA == A_current_mA)
    return String("a%20day");
  float availableAH = (((float)calculateBatteryPercentage()) / 100.0) * BATTERY_RATED_CAPACITY;
  float requiredAH = BATTERY_RATED_CAPACITY - availableAH;
  int hoursToCharge = requiredAH / (A_current_mA - B_current_mA);
  if (hoursToCharge > 23 || hoursToCharge < 0)
    return String("a%20day");
  if ((int)hoursToCharge == 0)
    return String("soon");
  return String((int)hoursToCharge) + "%20hrs";
}

void checkAndsendToAlexaINAReadings() {
  if (INAChangeDetected) {
    //A
    Serial.print("A_Bus Voltage = ");
    Serial.print(A_busvoltage);
    Serial.println(" V");

    Serial.print("A_Shunt Voltage = ");
    Serial.print(A_shuntvoltage);
    Serial.println(" mV");

    Serial.print("A_Load Voltage = ");
    Serial.print(A_loadvoltage);
    Serial.println(" V");

    Serial.print("A_Load Current = ");
    Serial.print(A_current_mA);
    Serial.println(" mA");

    Serial.print("A_Load power = ");
    Serial.print(A_power_W);
    Serial.println(" W");

    //B
    Serial.print("B_Bus Voltage = ");
    Serial.print(B_busvoltage);
    Serial.println(" V");

    Serial.print("B_Shunt Voltage = ");
    Serial.print(B_shuntvoltage);
    Serial.println(" mV");

    Serial.print("B_Load Voltage = ");
    Serial.print(B_loadvoltage);
    Serial.println(" V");

    Serial.print("B_Load Current = ");
    Serial.print(B_current_mA);
    Serial.println(" mA");

    Serial.print("B_Load power = ");
    Serial.print(B_power_W);
    Serial.println(" W");

    String basicMessage = "Generating%20" + String((int)(A_power_W)) + "%20watts%20at%20" + String((int)A_loadvoltage) + "%20volts%20and%20draining%20" + String((float)(B_power_W)) + "%20watts%20at%20" + String((float)B_loadvoltage) + "%20volts";
    if (isBatterCurrentlyInDanger() || isBatteryOutOfDanagerButInCoolOff()) {
      basicMessage = "recovering%20for%20" + String((int)calculateCoolOffRemainingTimeInMinutes()) + "%20minutes%2E%20" + basicMessage;
    } else {
      if (isUnderLoad()) {
        if (isBatteryDraining())
          basicMessage = "depleting%2E%20" + basicMessage;
        else
          basicMessage = "charging%2E%20" + basicMessage;
      } else {
        basicMessage = "idle%2E%20" + basicMessage;
      }
    }
    sendSensorValueToAlexa("SolarPanel", basicMessage);
    INAChangeDetected = false;
  }
}

boolean isUnderLoad() {
  return B_current_mA - MIN_LOAD_CURRENT > 0;
}

int calculateCoolOffRemainingTimeInMinutes() {
  unsigned long currentTime = millis();
  return (turnOnBatteryAccessAt - currentTime) / (1000 * 60);
}

void loadBMEReadings() {
  bme_readTemperature = bme.readTemperature();
  bme_readPressure = bme.readPressure() / 100.0F;
  bme_readHumidity = bme.readHumidity();
  bme_readAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if ((abs(bme_readTemperature - prev_bme_readTemperature) > PRECISSION_TEMP) || (abs(bme_readHumidity - prev_bme_readHumidity) > PRECISSION_HUMID)) {
    BMEChangeDetected = true;
    prev_bme_readTemperature = bme_readTemperature;
    prev_bme_readHumidity = bme_readHumidity;
  }
}

void checkAndsendToAlexaBMEReadings() {
  if (BMEChangeDetected) {
    Serial.print("Temperature = ");
    Serial.print(bme_readTemperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme_readPressure);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme_readHumidity);
    Serial.println(" %");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme_readAltitude);
    Serial.println(" m");

    //sendSensorValueToAlexa("Weather", String(bme.readTemperature())+"%20degree%20celsius%2C%20humidity%20is%20"+String(bme.readHumidity())+"%25%2C%20pressure%20is%20"+String(bme.readPressure() / 100.0F)+"%20hectopascal%2C%20altitude%20is%20"+String(bme.readAltitude(SEALEVELPRESSURE_HPA))+"%20meters");
    sendSensorValueToAlexa("Weather", String((int)bme_readTemperature) + "%20degree%20celsius%20at%20" + String((int)bme_readHumidity) + "%25%20humidity");
    BMEChangeDetected = false;
  }
}

void sendSensorValueToAlexa(String name, String reading) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String fullUrl = "https://home-automation.vadrin.com/droid/" + DROID_ID + "/upsert/intent/" + name + "/reading/" + reading;
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

void wifiSetup() {
  // Set WIFI module to STA mode
  WiFi.mode(WIFI_STA);

  // Connect
  Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Connected!
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void fetchAndLoadCurrentTimeFromWeb() {
  HTTPClient http;
  WiFiClient wifiClient;
  if (http.begin(wifiClient, WORLD_TIME_API)) {
    int httpCode = http.GET();

    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();

        // Parse the JSON response to extract the time and date
        int location = payload.indexOf("datetime");
        if (location != -1) {
          String time = payload.substring(location + 11, location + 30);  // Extract the time part from the response
          Serial.println(time);
          int year = time.substring(0, 4).toInt();
          int month = time.substring(5, 7).toInt();
          int day = time.substring(8, 10).toInt();
          int hour = time.substring(11, 13).toInt();
          int minute = time.substring(14, 16).toInt();
          int second = time.substring(17, 19).toInt();
          Serial.println(year);
          Serial.println(month);
          Serial.println(day);
          Serial.println(hour);
          Serial.println(minute);
          Serial.println(second);


          // Set the fetched date and time to the internal clock
          setTime(hour, minute, second, day, month, year);  // Set time (HH, MM, SS, DD, MM, YYYY)
          Serial.println("Time fetched and set.");
        } else {
          Serial.println("Failed to connect to the time server3");
        }
      } else {
        Serial.println("Failed to connect to the time server2");
      }
    }
    http.end();
  } else {
    Serial.println("Failed to connect to the time server1");
  }
}