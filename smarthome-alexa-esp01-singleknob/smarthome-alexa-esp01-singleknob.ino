/*
 * Make sure you configure your wifi SSID & Password before loadig
 */ 
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
//#define ESPALEXA_ASYNC            //uncomment for async operation (can fix empty body issue)
//#define ESPALEXA_NO_SUBPAGE       //disable /espalexa status page
//#define ESPALEXA_DEBUG            //activate debug serial logging
//#define ESPALEXA_MAXDEVICES 15    //set maximum devices add-able to Espalexa
#include <Espalexa.h>
#include <RBDdimmer.h>//

#define outputPin  2
#define zerocross  1 // for boards with CHANGEBLE input pins

// Change this!!
const char* ssid = "XXXXXX";
const char* password = "YYYYYY";

//Dimmer initialization
dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
uint8_t percent = 100;

// prototypes
bool connectWifi();

//callback functions
void knobCallback(EspalexaDevice* dev);

bool wifiConnected = false;

Espalexa espalexa;

void setup()
{
  Serial.begin(115200);

  // initialize dimmer
  dimmer.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)

  // Initialise wifi connection
  wifiConnected = connectWifi();
  if(!wifiConnected){
    while (1) {
      Serial.println("Cannot connect to WiFi. Please check data and reset the ESP.");
      delay(2500);
    }
  }
  
  // Define your devices here. 
  espalexa.addDevice("Fan", knobCallback, EspalexaDeviceType::dimmable, 127); //Dimmable device, optional 4th parameter is beginning state (here fully on)
  espalexa.begin();
}
 
void loop()
{
 espalexa.loop();
 if(percent < 15){
  Serial.println("Too low setting. Shutting down!");
  digitalWrite(outputPin, LOW);
 }else{
  dimmer.setPower(percent); // setPower(0-100%);
 }
 delay(50);
}

//our callback functions
void knobCallback(EspalexaDevice* d) {
  if (d == nullptr) return;

  uint8_t brightness = d->getValue();
  percent = d->getPercent();
  uint8_t degrees = d->getDegrees(); //for heaters, HVAC, ...

  Serial.print("Received value from alexa ");
  Serial.print(brightness);
  Serial.println(".");

  Serial.print("Knob changed to ");
  Serial.print(percent);
  Serial.println("%");
}

// connect to wifi – returns true if successful or false if not
bool connectWifi(){
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
    if (i > 20){
      state = false; break;
    }
    i++;
  }
  Serial.println("");
  if (state){
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
