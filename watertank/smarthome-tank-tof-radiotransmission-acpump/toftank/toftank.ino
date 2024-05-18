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

#include <DeepSleep.h>
#include <NewPing.h>

#define TRANSMISSION_TYPE LORA  // RH, LORA - Change PINS Below
#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

const int RETRY_ATTEMPTS = 20;
const String TANK_NAME = "GTSPureWaterTank";

//Dont touch below code
#define RH 1
#define LORA 2
#ifdef TRANSMISSION_TYPE
#if TRANSMISSION_TYPE == RH
#include "rh.h"                   // Include and use the RH_ASK display library
#elif TRANSMISSION_TYPE == LORA       // Include and use the LoRa display library
#include "lor.h"
#else
#error "Invalid library selection."
#endif
#else
#error "Library selection not defined."
#endif
float battVolts;   // made global for wider avaliblity throughout a sketch if needed, example for a low voltage alarm, etc value is volts X 100, 5 vdc = 500

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
DeepSleep deepSleep;

void setup()
{
  Serial.begin(9600);  // Debugging only
  Serial.print("Setting up Transmission");
  setupTransmission();
}

void loop()
{
  for (int i = 0; i <= 3; i++) battVolts = getBandgap(); //4 readings required for best stable value?
  /*
  Serial.print("Battery Vcc volts =  ");
  Serial.println(battVolts / 100);
  Serial.print("Analog pin 0 voltage = ");
  Serial.println(map(analogRead(0), 0, 1023, 0, battVolts));
  Serial.println();
  */
  //get average distance and VCC
  float distance = getMeanDistance();
  float battVolts = getMeanVcc();


  //send the message
  String distanceString = String(distance);
  //const char *msg = distanceString.c_str();
  String VccString = String(battVolts);
  String totalMessage = distanceString +","+VccString+","+TANK_NAME;
  transmitUsingTransmitter(totalMessage);
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


float getMeanVcc() {
  float toAverageVcc = 0;
  int validCounts = 0;
  for (int i = 0; i < 5; i++) {
    float thisVcc = battVolts;
    if (thisVcc != 0) {
      toAverageVcc = toAverageVcc + thisVcc;
      validCounts++;
    }
    delay(50);
  }
  if (validCounts > 0)
    return toAverageVcc / validCounts;
  else
    return 0;
}

int getBandgap(void)
{
  // For 168/328 boards
  const long InternalReferenceVoltage = 1050L;  // Adust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref.
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  // Start a conversion
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
  // Scale the value
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L;
  return results;
}
