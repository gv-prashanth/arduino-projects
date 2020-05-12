/*
  Automtic Water Level Controller

  The concept:
   Turns ON and OFF a sumpMotor pump Automtically by sensing presense of water in Overhead Tank and Sump.
   Using 433MHz RF Transmitter and Receiver modules for wireless link, along with
   HT12E (Encoder)and HT12D (Decoder) ICs for water level Data communiction and finally
   Arduino as logic controller to drive a sumpMotor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank bottom water level sensor attached to pin 2 with 10k resistor to ground
   Overhed tank top water level sensor attached to pin 3 with 10k resistor to ground
   Sump bottom water level Capasitor sensor attached to pin A4 - A5
   sumpMotorTriggerPin driver attached to pin 5  with 10k resistor to ground
   Valid transmition Indicates that the transmitter is alive! attached to pin A0 with 10k resistor to ground
*/
#include <Capacitor.h> //Download from https://github.com/codewrite/arduino-capacitor

//Pin Configurations
const int overheadTransmissionBottomPin = 9; // Over head tank water Minimum level
const int overheadTransmissionTopPin = 10; // Over head tank water Maximum level
const int overheadTransmissionValidityPin = A0; // Valid transmition  pin
const int sumpCapacitorSensorPin1 = A5; // Sump camapcitor sensor pin1
const int sumpCapacitorSensorPin2 = A4; // Sump camapcitor sensor pin2
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpLevelIndicatorPin = 11; // Sump water level led pin

//Functional Configurations
const float MAX_CAP_VALUE_IN_AIR = 3000;//(in pF)
const unsigned long WAIT_TIME = 10000; // in milliseconds

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime;
boolean cached_doesOverheadBottomHasWater;
boolean cached_doesOverheadTopHasWater;
boolean cached_doesSumpHasWater;
unsigned long mostRecentSumpFluctuationTime;
Capacitor capSensor(sumpCapacitorSensorPin1,sumpCapacitorSensorPin2);//One Pin should be analog atleast

void setup()
{
  // initialize serial communication with computer:
  Serial.begin(9600);

  // initialize the sumpMotorTriggerPin pin, SumpLevelIndicatorPinPin as Output
  pinMode(sumpMotorTriggerPin, OUTPUT);
  pinMode(SumpLevelIndicatorPin, OUTPUT);

  // initialize the overheadTransmissionBottomPinPin, overheadTransmissionTopPinPin as an input
  pinMode(overheadTransmissionBottomPin, INPUT);
  pinMode(overheadTransmissionTopPin, INPUT);

  // initialize all the necessary readings
  lastSuccesfulOverheadTransmissionTime = millis();
  cached_doesOverheadBottomHasWater = true;
  cached_doesOverheadTopHasWater = true;
  cached_doesSumpHasWater = false;
  mostRecentSumpFluctuationTime = millis();

}

void loop()
{
  // read values from all sensors
  float sumpCapacitance = capSensor.Measure();// Measure the capacitance (in pF)
  Serial.print("Sump capacitance is ");
  Serial.println(sumpCapacitance);
  indicateSumpLevel(sumpCapacitance);
  LoadAndCacheOverheadTransmissions();

  //Based on read values decide to switch on or off
  if (!isConnectionWithinTreshold()) {
    Serial.println("No strong signal from overhead. Shutting down!");
    digitalWrite(sumpMotorTriggerPin, LOW);
  } else if (!doesSumpHasWaterConsideringFluctuations(sumpCapacitance)) {
    Serial.println("Out of water in sump. Shutting down!");
    digitalWrite(sumpMotorTriggerPin, LOW);
  } else if (cached_doesOverheadTopHasWater) {
    Serial.println("Overhead tank is filled. Shutting down!");
    digitalWrite(sumpMotorTriggerPin, LOW);
  } else if (!cached_doesOverheadBottomHasWater) {
    Serial.println("No water in overhead tank. Switching ON!");
    digitalWrite(sumpMotorTriggerPin, HIGH);
  } else {
    Serial.println("Water level is between Bottom & Top. Lets leave the motor is whatever state it is in.");
  }
}

boolean isConnectionWithinTreshold() {
  unsigned long currentTime = millis();
  return currentTime - lastSuccesfulOverheadTransmissionTime < WAIT_TIME ;
}

void LoadAndCacheOverheadTransmissions() {
  boolean isOverheadTransmissionValid = digitalRead(overheadTransmissionValidityPin);
  if (isOverheadTransmissionValid) {
    cached_doesOverheadBottomHasWater = digitalRead(overheadTransmissionBottomPin);
    cached_doesOverheadTopHasWater = digitalRead(overheadTransmissionTopPin);
    lastSuccesfulOverheadTransmissionTime = millis();
  } else {
    Serial.println("Didnt get any signal from overhead. Will check again in sometime. Untill then use the cached values.");
  }

}

void indicateSumpLevel(float capacitance) {
  if (doesSumpHasWaterConsideringFluctuations(capacitance))
    digitalWrite(SumpLevelIndicatorPin, HIGH);
  else
    digitalWrite(SumpLevelIndicatorPin, LOW);
}

boolean doesSumpHasWaterConsideringFluctuations(float capacitance) {
  boolean current_doesSumpHasWater = (capacitance > MAX_CAP_VALUE_IN_AIR);
  if(current_doesSumpHasWater != cached_doesSumpHasWater)
    mostRecentSumpFluctuationTime = millis(); //there is a fluctuation
  cached_doesSumpHasWater = current_doesSumpHasWater;
  unsigned long currentTime = millis();
  if(currentTime - mostRecentSumpFluctuationTime > WAIT_TIME)
    return current_doesSumpHasWater;
  else
    return false;
}
