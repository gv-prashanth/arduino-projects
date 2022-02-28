#include <Arduino.h>

#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SNoDAC.h"

#include "RTClib.h"

#include "headLightWarning.h"
#include "errorWarning.h"
#include "welcome.h"
#include "handBrakesWarning.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing ESP reset pin)
const int NO_OF_TIMES_TO_REPEAT_ALERT = 3;
const int HEAD_LIGHTS_BEFORE = 6;//AM
const int HEAD_LIGHTS_AFTER = 18;//PM
const int AUDIO_END_DELAY = 2000;//ms
const int DISPLAY_SWITCH_DURATION = 10000;//ms

RTC_DS3231 rtc;
AudioGeneratorWAV *wav;
AudioFileSourcePROGMEM *file;
AudioOutputI2SNoDAC *out;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
boolean audioPlaying, greeted, rtcError, displayError;
int headLightsNotifiedCount, handBrakesNotifiedCount, errorNotifiedCount;
int screenToDisplay;
unsigned long lastDisplayChange;

void setup()
{
#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif
  Serial.begin(74880);
  audioPlaying = false;
  greeted = false;
  rtcError = false;
  displayError = false;
  headLightsNotifiedCount = 0;
  handBrakesNotifiedCount = 0;
  errorNotifiedCount = 0;
  screenToDisplay = 0;
  lastDisplayChange = millis();

  audioLogger = &Serial;
  out = new AudioOutputI2SNoDAC();
  wav = new AudioGeneratorWAV();

  int retry = 0;
  while (! rtc.begin()) {
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
  } else {
    display.clearDisplay();
    displaySequentially();
  }
}

void loop()
{
  if (audioPlaying) {
    continueAudioPlayer();
    return;
  } else {
    if (welcomeRequired()) {
      AudioFileSourcePROGMEM *welcomeFile = new AudioFileSourcePROGMEM( welcome, sizeof(welcome) );
      startAudioPlayer(welcomeFile);
      greeted = true;
      Serial.println("Welcome!");
    } else if (inError() && (errorNotifiedCount < NO_OF_TIMES_TO_REPEAT_ALERT)) {
      AudioFileSourcePROGMEM *errorFile = new AudioFileSourcePROGMEM( errorWarning, sizeof(errorWarning) );
      startAudioPlayer(errorFile);
      errorNotifiedCount++;
      Serial.println("Error!");
    } else if (inError()) {
      //Do Nothing! Already notified that there is error. Please restart...
    } else if (handBrakesReleaseRequired() && (handBrakesNotifiedCount < NO_OF_TIMES_TO_REPEAT_ALERT)) {
      AudioFileSourcePROGMEM *handBrakesFile = new AudioFileSourcePROGMEM( handBrakesWarning, sizeof(handBrakesWarning) );
      startAudioPlayer(handBrakesFile);
      handBrakesNotifiedCount++;
      Serial.println("Hand Brakes!");
    } else if (headLightsRequired() && (headLightsNotifiedCount < NO_OF_TIMES_TO_REPEAT_ALERT)) {
      AudioFileSourcePROGMEM *headLightFile = new AudioFileSourcePROGMEM( headLightWarning, sizeof(headLightWarning) );
      startAudioPlayer(headLightFile);
      headLightsNotifiedCount++;
      Serial.println("Head Lights!");
    } else {
      //No essential notifications at this point of time. May be play some fun notifications.
      displaySequentially();
    }
  }
}

boolean inError() {
  return rtcError || displayError;
}

boolean welcomeRequired() {
  return !greeted;
}

boolean handBrakesReleaseRequired() {
  return (handBrakesNotifiedCount < NO_OF_TIMES_TO_REPEAT_ALERT);
}

boolean headLightsRequired() {
  DateTime time = rtc.now();
  String hms = time.timestamp(DateTime::TIMESTAMP_TIME);   //captures first data String
  int ind1 = hms.indexOf(':');  //finds location of first ,
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
  unsigned long currentTime = millis();
  if (currentTime - lastDisplayChange > DISPLAY_SWITCH_DURATION) {
    screenToDisplay++;
    if (screenToDisplay > 1)
      screenToDisplay = 0;
    lastDisplayChange = currentTime;
  }
  if (screenToDisplay == 0)
    displayTime();
  if (screenToDisplay == 1)
    displayTemperature();
}

void displayTemperature() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(7);
  display.setCursor(2, 8);
  int temp = rtc.getTemperature();
  display.print(temp);
  display.setTextSize(3);
  display.print((char)247); // degree symbol
  display.setTextSize(4);
  display.print("C");
  display.display();
}

void displayDateTime() {
  DateTime dateTime = rtc.now();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("TIME");
  display.setCursor(0, 10);
  display.setTextSize(2);
  display.print(dateTime.hour());
  display.print(":");
  display.print(dateTime.minute());
  display.print(":");
  display.print(dateTime.second());
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("DATE");
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(dateTime.day());
  display.print("/");
  display.print(dateTime.month());
  display.print("/");
  display.print(dateTime.year());
  display.display();
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
  if(secs%2==0)
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
