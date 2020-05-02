/*
  Automtic Water Level Controller:

  The concept:
   Turns ON and OFF a Motor pump Automtically by sensing presense of water in Overhead Tank and Sump.
   Using 433MHz RF Transmitter and Receiver modules for wireless link, along with
   HT12E (Encoder)and HT12D (Decoder) ICs for water level Data communiction and finally
   Arduino as logic controller to drive a Motor Pump. Pump is  connected EM Relay mounted on Power supply unit.
   Overhed tank bottom water level sensor attached to pin 2 with 10k resistor to ground
   Overhed tank top water level sensor attached to pin 3 with 10k resistor to ground
   Sump bottom water level Capasitor sensor attached to pin A4 - A5
   Motor driver attached to pin 5  with 10k resistor to ground
   Valid transmition  (VT){Indicating that the transmitter is alive!} attached to pin A0  with 10k resistor to ground

  The Challenge:
   The VT pin is the only link to ensure the transmitter is sending signals
   Unfortunately VT pin is not HIGH always, voltage is not steady varying constantly.
   To ensure the stability and to avoid errors We need to take samples of varrying voltage
   and define the number of samples to keep track.  Higher the number of samples,
   more accurate readings will be smoothened, but slower response to the input.  Using a constant rather than a normal variable lets
   this value to determine the size of the readings array.
*/

const int OUT_PIN = A5;
const int  IN_PIN = A4;
const float IN_STRAY_CAP_TO_GND = 24.48;
const float IN_CAP_TO_GND  = IN_STRAY_CAP_TO_GND;
const float R_PULLUP = 34.8;
const int MAX_ADC_VALUE = 1023;
const float MAX_CAP_VALUE_IN_AIR = 0.3;//originally 0.28

int VT = A0;                     // Valid transmition  pin:
const int Motor =  8;             // Motor pump driver pin:
const int OverheadLevelLow = 9;      // Over head tank water Minimum level:
const int OverheadLevelHigh = 10;    // Over head tank water Maximum level:
const int SumpLevelLow = 11;        // Sump water Minimum level:


const int numReadings = 20;         // Define the number of samples to keep track of                                         ;
int readings[numReadings];         // the readings from the analog input
int readIndex = 0;                // the index of the current reading
int total = 0;                   // the running total
int average = 0;                // the average


void setup()
{

  pinMode(OUT_PIN, OUTPUT);
  pinMode(IN_PIN, OUTPUT);
  pinMode(SumpLevelLow, OUTPUT);

  // initialize serial communication with computer:
  Serial.begin(9600);

  // initialize the Motor pin as Output:
  pinMode(Motor, OUTPUT);

  // initialize the OverheadLevelLowPin, OverheadLevelHighPin, SumpLevelLowPin, as an input:
  pinMode(OverheadLevelLow, INPUT);
  pinMode(OverheadLevelHigh, INPUT);


  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop()
{
  float capacitance = measureCapacitanceInNanoFarad();
  Serial.println(capacitance, 2);
  if (capacitance < MAX_CAP_VALUE_IN_AIR)
  {
    digitalWrite(SumpLevelLow, LOW);
  }
  else
  {
    digitalWrite(SumpLevelLow, HIGH);
  }
  delay(1000);
  {

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = analogRead(VT);
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average = total / numReadings;
    // send it to the computer as ASCII digits
    Serial.println(average);
    delay(100);        // delay in between reads for stability


    // read the state of sensor INPUTS:

    int OverheadLevelLowState = digitalRead(OverheadLevelLow);
    int OverheadLevelHighState = digitalRead(OverheadLevelHigh);
    // int SumpLevelLowState = digitalRead(SumpLevelLow);


    //check if the Over head tnak water level is low(Minimum) and water in Sump is above Minimum level.
    // ie if transmitter sending signls and OverheadLevelLow state is LOW and sumplevelLow state is HIGH:

    if ( average > 1 && OverheadLevelLowState == LOW && capacitance > MAX_CAP_VALUE_IN_AIR)

      // turn motor ON :
      digitalWrite(Motor, HIGH);

    // If transmitter not sending signls OR OverheadLevelHigh state is HIGH (Overhed tank is full
    // OR sumplevelLow state is LOW(Sump water level below minimum) :

    if (average < 1 || OverheadLevelHighState == HIGH || capacitance < MAX_CAP_VALUE_IN_AIR)

      // turn motor OFF:
      digitalWrite(Motor, LOW);
  }

}

float measureCapacitanceInNanoFarad() {
  pinMode(IN_PIN, INPUT);
  digitalWrite(OUT_PIN, HIGH);
  int val = analogRead(IN_PIN);
  digitalWrite(OUT_PIN, LOW);

  if (val < 1000)
  {
    pinMode(IN_PIN, OUTPUT);
    return ((float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val)) / 1000;
  }
  else
  {
    pinMode(IN_PIN, OUTPUT);
    delay(1);
    pinMode(OUT_PIN, INPUT_PULLUP);
    unsigned long u1 = micros();
    unsigned long t;
    int digVal;
    do
    {
      digVal = digitalRead(OUT_PIN);
      unsigned long u2 = micros();
      t = u2 > u1 ? u2 - u1 : u1 - u2;
    } while ((digVal < 1) && (t < 400000L));
    pinMode(OUT_PIN, INPUT);
    val = analogRead(OUT_PIN);
    digitalWrite(IN_PIN, HIGH);
    int dischargeTime = (int)(t / 1000L) * 5;
    delay(dischargeTime);
    pinMode(OUT_PIN, OUTPUT);
    digitalWrite(OUT_PIN, LOW);
    digitalWrite(IN_PIN, LOW);
    return -(float)t / R_PULLUP
           / log(1.0 - (float)val / (float)MAX_ADC_VALUE);
  }
}
