/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank.
   Using 433MHz RF Transmitter and Receiver modules for wireless link, along with RH_ASK to transmit the water level reading.
   Make sure you connect the RF Receiver to PIN 11 & RF Transmitter to PIN 12
   Arduino as logic controller to drive a sumpMotor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank is connected to a TOF distance sensor (Ultrasonic sensor)
   sumpMotorTriggerPin driver attached to pin 5  with 10k resistor to ground
*/

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif
#include <DeepSleep.h>
#include <NewPing.h>

#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

RH_ASK driver;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
DeepSleep deepSleep;

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
}

void loop()
{
  //get average distance
  float distance = getMeanDistance();

  //send the message
  String distanceString = String(distance);
  const char *msg = distanceString.c_str();
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  //Serial.print(distance); Serial.println(" cm");delay(100);

  //sleep for 8 sec
  deepSleep.sleepForEightSecondsUnlessInterrupted();
}

float getMeanDistance() {
  float toAverageDistance = 0;
  int validCounts = 0;
  for (int i = 0; i < 5; i++) {
    float thisDist = sonar.ping_cm();
    if (thisDist != 0) {
      toAverageDistance = toAverageDistance + thisDist;
      validCounts++;
    }
    delay(50);
  }
  if (validCounts > 0)
    return toAverageDistance / validCounts;
  else
    return 0;
}
