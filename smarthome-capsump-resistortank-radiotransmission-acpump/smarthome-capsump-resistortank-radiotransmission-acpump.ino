/*
  Automtic Water Level Controller:

  The concept:
   Turns ON and OFF a sumpMotorTriggerPin pump Automtically by sensing presense of water in Overhead Tank and Sump.
   Using 433MHz RF Transmitter and Receiver modules for wireless link, along with
   HT12E (Encoder)and HT12D (Decoder) ICs for water level Data communiction and finally
   Arduino as logic controller to drive a sumpMotorTriggerPin Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank bottom water level sensor attached to pin 2 with 10k resistor to ground
   Overhed tank top water level sensor attached to pin 3 with 10k resistor to ground
   Sump bottom water level Capasitor sensor attached to pin A4 - A5
   sumpMotorTriggerPin driver attached to pin 5  with 10k resistor to ground
   Valid transmition  (overheadTransmissionValidityPin){Indicating that the transmitter is alive!} attached to pin A0  with 10k resistor to ground

  The Challenge:
   The overheadTransmissionValidityPin pin is the only link to ensure the transmitter is sending signals
   Unfortunately overheadTransmissionValidityPin pin is not HIGH always, voltage is not steady varying constantly.
   To ensure the stability and to avoid errors We need to take samples of varrying voltage
   and define the number of samples to keep track.  Higher the number of samples,
   more accurate readings will be smoothened, but slower response to the input.  Using a constant rather than a normal variable lets
   this value to determine the size of the readings array.
*/

//Pin Configurations
const int overheadTransmissionLowPin = 9; // Over head tank water Minimum level
const int overheadTransmissionHighPin = 10; // Over head tank water Maximum level
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
const int NUM_READINGS = 40; // Define the number of samples to keep track of                                         ;

//Dont touch below stuff
int readings[NUM_READINGS]; // the readings from the analog input
int readIndex = 0; // the index of the current reading
int total = 0; // the running total
int average = 0; // the average

void setup()
{
  // initialize serial communication with computer:
  Serial.begin(9600);

  // initialize the sumpMotorTriggerPin pin, SumpLevelIndicatorPinPin as Output:
  pinMode(sumpMotorTriggerPin, OUTPUT);
  pinMode(SumpLevelIndicatorPin, OUTPUT);

  // initialize the overheadTransmissionLowPinPin, overheadTransmissionHighPinPin as an input:
  pinMode(overheadTransmissionLowPin, INPUT);
  pinMode(overheadTransmissionHighPin, INPUT);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < NUM_READINGS; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop()
{
  // read values from all sensors
  float sumpCapacitance = measureCapacitanceInNanoFarad();
  int overheadTransmissionStrength = analogRead(overheadTransmissionValidityPin);
  boolean overheadLevelLow = digitalRead(overheadTransmissionLowPin);
  boolean overheadLevelHigh = digitalRead(overheadTransmissionHighPin);

  //switch on or off the motor
  if (sumpHasWater(sumpCapacitance) && isValidTransmission(overheadTransmissionStrength) && overheadLevelLow)
    digitalWrite(sumpMotorTriggerPin, HIGH);
  else if (!sumpHasWater(sumpCapacitance) || !isValidTransmission(overheadTransmissionStrength) || overheadLevelHigh)
    digitalWrite(sumpMotorTriggerPin, LOW);
  else
    Serial.println("No need to do anything");

  //Take a break for a second
  delay(1000);
}

boolean isValidTransmission(int overheadTransmissionStrength) {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = overheadTransmissionStrength;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= NUM_READINGS) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  average = total / NUM_READINGS;
  // send it to the computer as ASCII digits
  Serial.println(average);
  return average > 1;
}

boolean sumpHasWater(float capacitance) {
  Serial.println(capacitance, 2);
  if (capacitance < MAX_CAP_VALUE_IN_AIR)
  {
    digitalWrite(SumpLevelIndicatorPin, LOW);
    return false;
  }
  else
  {
    digitalWrite(SumpLevelIndicatorPin, HIGH);
    return true;
  }
}

float measureCapacitanceInNanoFarad() {
  //First lets set both to output
  pinMode(sumpCapacitorSensorPin1, OUTPUT);
  pinMode(sumpCapacitorSensorPin2, OUTPUT);

  //Now check with 2 as input. Revert back to output once ur done.
  pinMode(sumpCapacitorSensorPin2, INPUT);
  digitalWrite(sumpCapacitorSensorPin1, HIGH);
  int val = analogRead(sumpCapacitorSensorPin2);
  digitalWrite(sumpCapacitorSensorPin1, LOW);
  pinMode(sumpCapacitorSensorPin2, OUTPUT);

  //Now check with 1 as input. Revert back to output once ur done.
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
