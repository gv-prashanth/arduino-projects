/*
   Make sure you configure your wifi SSID & Password before loading
   Make sure you use Board version 2.5.0
*/
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
//#define ESPALEXA_ASYNC            //uncomment for async operation (can fix empty body issue)
//#define ESPALEXA_NO_SUBPAGE       //disable /espalexa status page
//#define ESPALEXA_DEBUG            //activate debug serial logging
#define ESPALEXA_MAXDEVICES 1    //set maximum devices add-able to Espalexa
#include <Espalexa.h>  // you can download the library from https://github.com/Aircoookie/Espalexa
#include <IRsend.h>

// Change this!!
const char* ssid = "XXXXXX";
const char* password = "YYYYYY";

const int irLed = 2;

// prototypes
bool connectWifi();

//callback functions
void knobCallbackA(EspalexaDevice* dev);

bool wifiConnected = false;

Espalexa espalexa;

IRsend irsend(irLed);

void setup()
{
  Serial.begin(115200);

  // Initialise wifi connection
  wifiConnected = connectWifi();
  if (!wifiConnected) {
    while (1) {
      Serial.println("Cannot connect to WiFi. Please check data and reset the ESP.");
      delay(2500);
    }
  }

  // Define your first device here.
  espalexa.addDevice("AC", knobCallbackA, EspalexaDeviceType::dimmable, 127); //Dimmable device, optional 4th parameter is beginning state (here fully on)
  espalexa.begin();
  irsend.begin();
}

void loop()
{
  espalexa.loop();

}

//our callback functions
void knobCallbackA(EspalexaDevice* d) {
  if (d == nullptr) return;

  if (d->getLastChangedProperty() == EspalexaDeviceProperty::off) {
    Serial.println("Looks like switch off command was invoked");
    irsend.sendLG(0x88C0051);
  } else {
    if (d->getLastChangedProperty() == EspalexaDeviceProperty::on) {
      Serial.println("Looks like switch on command was invoked");
      irsend.sendLG(0x8800707);
    }
    if (d->getLastChangedProperty() == EspalexaDeviceProperty::bri) {
      uint8_t brightness = d->getValue();
      uint8_t percent = d->getPercent();
      uint8_t degrees = d->getDegrees(); //for heaters, HVAC, ...
      Serial.println("Looks like brightness command was invoked either with value or percentage or degree");
      Serial.print("Received value from alexa "); Serial.print(brightness); Serial.println(".");
      Serial.print("Received percentage from alexa "); Serial.print(percent); Serial.println("%");
      Serial.print("Received degree from alexa "); Serial.print(degrees); Serial.println("C");
    }
  }
}

// connect to wifi – returns true if successful or false if not
bool connectWifi() {
  bool state = true;
  int i = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.println("Connecting to WiFi");

  // Wait for connection
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (i > 20) {
      state = false; break;
    }
    i++;
  }
  Serial.println("");
  if (state) {
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("Connection failed.");
  }
  return state;
}
