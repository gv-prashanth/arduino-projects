#include <Arduino.h>

#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SNoDAC.h"

#include "RTClib.h"

#include "headLightWarning.h"
//#include "errorWarning.h"
//#include "welcome.h"
#include "handBrakesWarning.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "bsec.h"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing ESP reset pin)
const int NO_OF_TIMES_TO_REPEAT_ALERT = 3;
const int HEAD_LIGHTS_BEFORE = 6;          //AM
const int HEAD_LIGHTS_AFTER = 18;          //PM
const int AUDIO_END_DELAY = 2000;          //ms
const int DISPLAY_SWITCH_DURATION = 2000;  //ms
const int MAX_SCREENS = 4;                 //time, temp, humidity, aqi

Bsec iaqSensor;
RTC_DS3231 rtc;
AudioGeneratorWAV *wav;
AudioFileSourcePROGMEM *file;
AudioOutputI2SNoDAC *out;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
boolean audioPlaying, rtcError, displayError;
int headLightsNotifiedCount, handBrakesNotifiedCount;
int screenToDisplay;
unsigned long lastDisplayChange;
int temperature, humidity, aqi;

void setup() {
#ifndef ESP8266
  while (!Serial)
    ;  // wait for serial port to connect. Needed for native USB
#endif
  Serial.begin(115200);
  Wire.begin(4, 5);  // Wire.begin(SDA,SCL);
  audioPlaying = false;
  rtcError = false;
  displayError = false;
  headLightsNotifiedCount = 0;
  handBrakesNotifiedCount = 0;
  screenToDisplay = 0;
  lastDisplayChange = millis();

  audioLogger = &Serial;
  out = new AudioOutputI2SNoDAC();
  wav = new AudioGeneratorWAV();

  int retry = 0;
  while (!rtc.begin()) {
    if (retry > 10) {
      rtcError = true;
      break;
    }
    retry++;
    delay(500);
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2022, 2, 26, 3, 0, 0));

  //Wire.begin();
  // Address 0x3C for 128x64, you might need to change this value (use an I2C scanner)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayError = true;
  }

  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  String output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
}

void loop() {
  if (audioPlaying) {
    continueAudioPlayer();
    return;
  }
  if (rtcError) {
    Serial.println("RTC Error!");
    return;
  }
  if (displayError) {
    Serial.println("Display Error!");
    return;
  }
  if (handBrakesReleaseRequired() && (handBrakesNotifiedCount < NO_OF_TIMES_TO_REPEAT_ALERT)) {
    AudioFileSourcePROGMEM *handBrakesFile = new AudioFileSourcePROGMEM(handBrakesWarning, sizeof(handBrakesWarning));
    startAudioPlayer(handBrakesFile);
    handBrakesNotifiedCount++;
    Serial.println("Hand Brakes!");
    return;
  }
  if (headLightsRequired() && (headLightsNotifiedCount < NO_OF_TIMES_TO_REPEAT_ALERT)) {
    AudioFileSourcePROGMEM *headLightFile = new AudioFileSourcePROGMEM(headLightWarning, sizeof(headLightWarning));
    startAudioPlayer(headLightFile);
    headLightsNotifiedCount++;
    Serial.println("Head Lights!");
    return;
  }
  //No essential notifications at this point of time. May be play some fun notifications.
  displaySequentially();
}

boolean handBrakesReleaseRequired() {
  return (handBrakesNotifiedCount < NO_OF_TIMES_TO_REPEAT_ALERT);
}

boolean headLightsRequired() {
  DateTime time = rtc.now();
  String hms = time.timestamp(DateTime::TIMESTAMP_TIME);  //captures first data String
  int ind1 = hms.indexOf(':');                            //finds location of first ,
  String hrs = hms.substring(0, ind1);
  int hrsInt = hrs.toInt();
  return (hrsInt >= HEAD_LIGHTS_AFTER || hrsInt <= HEAD_LIGHTS_BEFORE);
}

void continueAudioPlayer() {
  if (wav->isRunning()) {
    if (!wav->loop()) wav->stop();
  } else {
    stopAudioPlayer();
  }
}

void startAudioPlayer(AudioFileSourcePROGMEM *fileToPlay) {
  audioPlaying = true;
  file = fileToPlay;
  wav->begin(file, out);
}

void stopAudioPlayer() {
  audioPlaying = false;
  delay(AUDIO_END_DELAY);
}

void displaySequentially() {
  identifyScreenToDisplay();
  if (screenToDisplay == 1)
    displayTime();
  else if (screenToDisplay == 2)
    displayTemperature();
  else if (screenToDisplay == 3)
    displayHumidity();
  else if (screenToDisplay == 4)
    displayAQI();
}

void identifyScreenToDisplay() {
  unsigned long currentTime = millis();
  if (currentTime - lastDisplayChange > DISPLAY_SWITCH_DURATION) {
    screenToDisplay++;
    if (screenToDisplay > MAX_SCREENS)
      screenToDisplay = 1;
    lastDisplayChange = currentTime;
  }
}

void displayTemperature() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  gatherBMEReadings();
  display.print(temperature);
  display.print((char)247);  // degree symbol
  display.print("C");
  display.display();
}

void displayHumidity() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  display.print(humidity);
  display.print("%");
  display.display();
}

void displayAQI() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  display.print(aqi);
  display.print("PPM");
  display.display();
}

void gatherBMEReadings() {
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) {  // If new data is available
    String output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);
  } else {
    checkIaqSensorStatus();
  }
}

void displayTime() {
  DateTime dateTime = rtc.now();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  String hrsString = "";
  int hrs = dateTime.hour();
  if (hrs < 10)
    hrsString.concat(String("0"));
  hrsString.concat(String(hrs));
  display.print(hrsString);
  int secs = dateTime.second();
  if (secs % 2 == 0)
    display.print(":");
  else
    display.print(" ");
  String minsString = "";
  int mins = dateTime.minute();
  if (mins < 10)
    minsString.concat(String("0"));
  minsString.concat(String(mins));
  display.print(minsString);
  display.display();
}


// Helper function definitions
void checkIaqSensorStatus(void) {
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      String output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      String output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      String output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      String output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void) {
  Serial.println("Stuck in bme loop since there is error");
}