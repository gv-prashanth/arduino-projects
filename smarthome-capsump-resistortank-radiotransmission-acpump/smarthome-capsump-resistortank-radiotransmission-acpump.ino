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
const int sumpCapacitorSensorPin1 = A5; // Sump camapcitor sensor pin1
const int sumpCapacitorSensorPin2 = A4; // Sump camapcitor sensor pin2
const int sumpMotorTriggerPin = 8; // sump pump driver pin
const int SumpLevelIndicatorPin = 11; // Sump water level led pin

//Functional Configurations
const float IN_CAP_TO_GND  = 24.48;
const float R_PULLUP = 34.8;
const int MAX_ADC_VALUE = 1023;
const float MAX_CAP_VALUE_IN_AIR = 0.3;//originally 0.28
const unsigned long DISCONNECT_WAIT_TIME = 5000; // in milliseconds

//Dont touch below stuff
unsigned long lastSuccesfulOverheadTransmissionTime;
boolean cached_doesOverheadBottomHasWater;
boolean cached_doesOverheadTopHasWater;

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

}

void loop()
{
  // read values from all sensors
  float sumpCapacitance = measureCapacitanceInNanoFarad();
  Serial.print("Sump capacitance is ");
  Serial.println(sumpCapacitance, 2);
  indicateSumpLevel(sumpCapacitance);
  LoadAndCacheOverheadTransmissions();

  //Based on read values decide to switch on or off
  if (!isConnectionWithinTreshold()) {
    Serial.println("No strong signal from overhead. Shutting down!");
    digitalWrite(sumpMotorTriggerPin, LOW);
  } else if (!sumpHasWater(sumpCapacitance)) {
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
  return currentTime - lastSuccesfulOverheadTransmissionTime < DISCONNECT_WAIT_TIME ;
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

boolean indicateSumpLevel(float capacitance) {
  if (sumpHasWater(capacitance))
    digitalWrite(SumpLevelIndicatorPin, HIGH);
  else
    digitalWrite(SumpLevelIndicatorPin, LOW);
}

boolean sumpHasWater(float capacitance) {
  return (capacitance > MAX_CAP_VALUE_IN_AIR);
}

float measureCapacitanceInNanoFarad() {
  //First lets set both to output
  pinMode(sumpCapacitorSensorPin1, OUTPUT);
  pinMode(sumpCapacitorSensorPin2, OUTPUT);

  //Now check with 2 as input. Revert back once ur done.
  pinMode(sumpCapacitorSensorPin2, INPUT);
  digitalWrite(sumpCapacitorSensorPin1, HIGH);
  int val = analogRead(sumpCapacitorSensorPin2);
  digitalWrite(sumpCapacitorSensorPin1, LOW);
  pinMode(sumpCapacitorSensorPin2, OUTPUT);

  //Now check with 1 as input. Revert back once ur done.
  if (val < 1000)
  {
    return ((float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val)) / 1000;
  }
  else
  {
    delay(1);
    pinMode(sumpCapacitorSensorPin1, INPUT_PULLUP);
    unsigned long u1 = micros();
    unsigned long t;
    int digVal;
    do
    {
      digVal = digitalRead(sumpCapacitorSensorPin1);
      unsigned long u2 = micros();
      t = u2 > u1 ? u2 - u1 : u1 - u2;
    } while ((digVal < 1) && (t < 400000L));
    pinMode(sumpCapacitorSensorPin1, INPUT);
    val = analogRead(sumpCapacitorSensorPin1);
    digitalWrite(sumpCapacitorSensorPin2, HIGH);
    int dischargeTime = (int)(t / 1000L) * 5;
    delay(dischargeTime);
    pinMode(sumpCapacitorSensorPin1, OUTPUT);
    digitalWrite(sumpCapacitorSensorPin1, LOW);
    digitalWrite(sumpCapacitorSensorPin2, LOW);
    return -(float)t / R_PULLUP
           / log(1.0 - (float)val / (float)MAX_ADC_VALUE);
  }
}
