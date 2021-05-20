/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank.
   Using 433MHz RF Transmitter and Receiver modules for wireless link, along with RH_ASK to transmit the water level reading.
   Make sure you connect the RF Receiver to PIN 11 & RF Transmitter to PIN 12
   Arduino as logic controller to drive a sumpMotor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank is connected to a PWM based TOF distance sensor
   sumpMotorTriggerPin driver attached to pin 5  with 10k resistor to ground
*/

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

RH_ASK driver;

//Pin Configurations
const int tofPWMPin = 5; // Over head tank water level measure TOF pin

void setup()
{
#ifdef RH_HAVE_SERIAL
  Serial.begin(9600);    // Debugging only
#endif
  if (!driver.init())
#ifdef RH_HAVE_SERIAL
    Serial.println("init failed");
#else
    ;
#endif
  pinMode(tofPWMPin, INPUT);
}

void loop()
{
  unsigned long duration = pulseIn(tofPWMPin, HIGH);
  float distance = duration / 100.0;
  String distanceString = String(distance);
  const char *msg = distanceString.c_str();
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  Serial.print(distance);Serial.println(" cm");
  delay(200);
}
