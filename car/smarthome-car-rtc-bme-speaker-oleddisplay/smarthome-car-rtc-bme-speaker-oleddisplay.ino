#include <Arduino.h>

#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SNoDAC.h"

#include "RTClib.h"

#include "headLightWarning.h"
//#include "errorWarning.h"
#include "welcome.h"
#include "handBrakesWarning.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <EEPROM.h>
#include "bsec.h"  //USE V1.6.x version of BSEC Library. HIGHER VERSION WONT WORK

/* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
    generic_18v_3s_4d
    generic_18v_3s_28d
    generic_18v_300s_4d
    generic_18v_300s_28d
    generic_33v_3s_4d
    generic_33v_3s_28d
    generic_33v_300s_4d
    generic_33v_300s_28d
*/

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

//#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000)  // 360 minutes - 4 times a day
#define STATE_SAVE_PERIOD UINT32_C(15 * 60 * 1000)  // Every 15 minutes

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing ESP reset pin)
const int NO_OF_TIMES_TO_REPEAT_ALERT = 3;
const int HEAD_LIGHTS_BEFORE = 6;          //AM
const int HEAD_LIGHTS_AFTER = 18;          //PM
const int AUDIO_END_DELAY = 2000;          //ms
const int DISPLAY_SWITCH_DURATION = 3000;  //ms
const int DELAY_START_DURATION = 30000;    //ms
const int MAX_SCREENS = 4;                 //time, temp, humidity, aqi, aqiAccuracy

RTC_DS3231 rtc;
AudioGeneratorWAV *wav;
AudioFileSourcePROGMEM *file;
AudioOutputI2SNoDAC *out;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
boolean audioPlaying, errorHappened, deviceGreeted;
int headLightsNotifiedCount, handBrakesNotifiedCount;
int screenToDisplay;
unsigned long lastDisplayChange;
int hrs, mins, secs, temperature, humidity, aqi, aqiAccuracy;
String output;

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = { 0 };
uint16_t stateUpdateCounter = 0;
// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);  // Wire.begin(SDA,SCL);
  loadDefaultValues();
  setupDisplay();
  delay(DELAY_START_DURATION);
  setupRTC();
  setupEPROMAndBME();
}

void displayError() {
  Serial.println("Err!");
  displayMessage("Error");
}

void loop() {
  gatherBMEReadings();
  gatherClockReadings();
  playNotificationIfRequired();
  displaySequentially();
}

void playNotificationIfRequired() {
  if (audioPlaying) {
    continueAudioPlayer();
    return;
  }
  if (!deviceGreeted) {
    AudioFileSourcePROGMEM *welcomeFile = new AudioFileSourcePROGMEM(welcome, sizeof(welcome));
    startAudioPlayer(welcomeFile);
    Serial.println("Welcome!");
    deviceGreeted = true;
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
  if (screenToDisplay == 0)
    displayError();
  else if (screenToDisplay == 1)
    displayTime();
  else if (screenToDisplay == 2)
    displayTemperature();
  else if (screenToDisplay == 3)
    displayHumidity();
  else if (screenToDisplay == 4)
    displayAQI();
}

void identifyScreenToDisplay() {
  if (errorHappened) {
    screenToDisplay = 0;
  } else {
    unsigned long currentTime = millis();
    if (currentTime - lastDisplayChange > DISPLAY_SWITCH_DURATION) {
      screenToDisplay++;
      if (screenToDisplay > MAX_SCREENS)
        screenToDisplay = 1;
      lastDisplayChange = currentTime;
    }
  }
}

void displayTemperature() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  display.print(temperature);
  display.print((char)247);  // degree symbol
  display.setCursor(80, 33);
  display.setTextSize(2);
  display.print("TEMP");
  displayLogo();
  display.display();
}

void displayHumidity() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  display.print(humidity);
  display.print("%");
  display.setCursor(80, 33);
  display.setTextSize(2);
  display.print("HUMD");
  displayLogo();
  display.display();
}

void displayAQI() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  display.print(aqi);
  display.setCursor(80, 33);
  display.setTextSize(2);
  display.print("AIRQ");
  display.setCursor(120, 0);
  display.setTextSize(1);
  display.print(aqiAccuracy);
  displayLogo();
  display.display();
}

void displayMessage(String message) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  display.print(message);
  displayLogo();
  display.display();
}

void displayLogo() {
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(5, 0);
  display.print("GTS");
}

void gatherBMEReadings() {
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) {  // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    Serial.println(output);
    temperature = iaqSensor.temperature;
    humidity = iaqSensor.humidity;
    aqi = iaqSensor.iaq;
    aqiAccuracy = iaqSensor.iaqAccuracy;
    updateState();
  } else {
    checkIaqSensorStatus();
  }
}

void gatherClockReadings() {
  DateTime dateTime = rtc.now();
  hrs = dateTime.hour();
  mins = dateTime.minute();
  secs = dateTime.second();
}

void displayTime() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 20);
  String hrsString = "";
  if (hrs < 10)
    hrsString.concat(String("0"));
  hrsString.concat(String(hrs));
  display.print(hrsString);
  if (secs % 2 == 0)
    display.print(":");
  else
    display.print(" ");
  String minsString = "";
  if (mins < 10)
    minsString.concat(String("0"));
  minsString.concat(String(mins));
  display.print(minsString);
  displayLogo();
  display.display();
}

// Helper function definitions
void checkIaqSensorStatus(void) {
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  } else {
    errorHappened = false;
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  } else {
    errorHappened = false;
  }
  iaqSensor.status = BSEC_OK;
}

void errLeds(void) {
  errorHappened = true;
  Serial.println("Stuck in bme loop since there is error");
}

void loadState(void) {
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

void updateState(void) {
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}

void loadDefaultValues() {
  deviceGreeted = false;
  audioPlaying = false;
  errorHappened = false;
  headLightsNotifiedCount = 0;
  handBrakesNotifiedCount = 0;
  screenToDisplay = 0;
  lastDisplayChange = millis();
  audioLogger = &Serial;
  out = new AudioOutputI2SNoDAC();
  wav = new AudioGeneratorWAV();
}

void setupRTC() {
  int retry = 0;
  boolean rtcSuccss = true;
  while (!rtc.begin()) {
    if (retry > 10) {
      errorHappened = true;
      rtcSuccss = false;
      break;
    }
    errorHappened = false;
    retry++;
    delay(500);
  }

  // When time needs to be set on a new device, or after a power loss, the
  // following line sets the RTC to the date & time this sketch was compiled
  if (rtcSuccss && rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void setupDisplay() {
  //Wire.begin();
  // Address 0x3C for 128x64, you might need to change this value (use an I2C scanner)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    errorHappened = true;
  }
  errorHappened = false;
  displayMessage("Hello");
}

void setupEPROMAndBME() {
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);  // 1st address for the length

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  loadState();

  bsec_virtual_sensor_t sensorList[7] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%]";
  Serial.println(output);
}