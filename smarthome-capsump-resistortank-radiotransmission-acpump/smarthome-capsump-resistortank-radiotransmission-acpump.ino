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
//Pin Configurations
const int overheadTransmissionBottomPin = 9; // Over head tank water Minimum level
const int overheadTransmissionTopPin = 10; // Over head tank water Maximum level
const int overheadTransmissionValidityPin = A0; // Valid transmition  pin
const int sumpCapacitorSensorPin = A5; // Sump camapcitor sensor pin
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpLevelIndicatorPin = 11; // Sump water level led pin

//Functional Configurations
const unsigned long TRANSMISSION_TRESHOLD_TIME = 10000; // in milliseconds
const unsigned long PROTECTION_BETWEEN_SWITCH_OFF_ON = 1800000; //in milliseconds
const float MAX_CAP_ANALOG_READING_IN_WATER = 380;//(in range from 0 to 1024)

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime, lastSwitchOffTime;
boolean cached_doesOverheadBottomHasWater;
boolean cached_doesOverheadTopHasWater;
boolean firstTimeStarting;

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
  lastSwitchOffTime = millis();
  cached_doesOverheadBottomHasWater = true;
  cached_doesOverheadTopHasWater = true;
  firstTimeStarting = true;
}

void loop()
{
  // read values from all sensors
  float sumpCapacitanceReading = analogRead(sumpCapacitorSensorPin);
  Serial.println("Sump capacitance reading is " + String(sumpCapacitanceReading));
  indicateSumpLevel(sumpCapacitanceReading);
  loadAndCacheOverheadTransmissions();

  //Based on read values decide to switch on or off
  if (!isConnectionWithinTreshold()) {
    Serial.println("No strong signal from overhead. Shutting down!");
    switchOffMotor();
  } else if (!doesSumpHasWater(sumpCapacitanceReading)) {
    Serial.println("Out of water in sump. Shutting down!");
    switchOffMotor();
  } else if (cached_doesOverheadTopHasWater) {
    Serial.println("Overhead tank is filled. Shutting down!");
    switchOffMotor();
  } else if (!cached_doesOverheadBottomHasWater) {
    Serial.println("No water in overhead tank. Also sump has water. Switching ON!");
    switchOnMotor();
  } else {
    Serial.println("Water level is between Bottom & Top. Lets leave the motor is whatever state it is in.");
  }
}

void switchOffMotor() {
  lastSwitchOffTime = millis();
  digitalWrite(sumpMotorTriggerPin, LOW);
}

void switchOnMotor() {
  unsigned long currentTime = millis();
  if (firstTimeStarting) {
    firstTimeStarting = false;
    digitalWrite(sumpMotorTriggerPin, HIGH);
  } else if (currentTime - lastSwitchOffTime < PROTECTION_BETWEEN_SWITCH_OFF_ON) {
    Serial.println("Very recently switched Off. Please wait for protection time.");
  } else {
    digitalWrite(sumpMotorTriggerPin, HIGH);
  }
}

boolean isConnectionWithinTreshold() {
  unsigned long currentTime = millis();
  return currentTime - lastSuccesfulOverheadTransmissionTime < TRANSMISSION_TRESHOLD_TIME ;
}

void loadAndCacheOverheadTransmissions() {
  boolean isOverheadTransmissionValid = digitalRead(overheadTransmissionValidityPin);
  if (isOverheadTransmissionValid) {
    cached_doesOverheadBottomHasWater = digitalRead(overheadTransmissionBottomPin);
    cached_doesOverheadTopHasWater = digitalRead(overheadTransmissionTopPin);
    lastSuccesfulOverheadTransmissionTime = millis();
  } else {
    Serial.println("Didnt get any signal from overhead. Will check again in sometime. Untill then use the cached values.");
  }

}

void indicateSumpLevel(float sumpCapacitanceReading) {
  if (doesSumpHasWater(sumpCapacitanceReading))
    digitalWrite(SumpLevelIndicatorPin, HIGH);
  else
    digitalWrite(SumpLevelIndicatorPin, LOW);
}

boolean doesSumpHasWater(float sumpCapacitanceReading) {
  return (sumpCapacitanceReading < MAX_CAP_ANALOG_READING_IN_WATER);
}
