#include <VoltageSensor.h>

const int batteryVoltageSensePin = A0;//A2 incase you want to detect from dedicated pin. -1 incase you want to detect from vcc.

const float smallR = 10000.0;//Ohms. It is Voltage sensor smaller Resistance value. Usually the one connected to ground.
const float bigR = 15000.0;//Ohms. It is Voltage sensor bigger Resistance value. Usually the one connected to sense.

VoltageSensor batteryVoltageSensor(batteryVoltageSensePin, smallR, bigR);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  digitalWrite(13, LOW);
}

void loop() {
    float voltage = batteryVoltageSensor.senseVoltage();
    Serial.println("Battery Voltage: " + String(voltage));
    if(voltage < 10){
      digitalWrite(LED_BUILTIN, HIGH);
    }else{
      digitalWrite(LED_BUILTIN, LOW);
    }
    delay(1000);
}
