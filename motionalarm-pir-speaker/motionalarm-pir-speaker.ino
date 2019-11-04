/*
 * PIR sensor tester
 */
const int speakerPin = 4;
const int talkFrequency = 2000;//frequency in Hz
int ledPin = 13;                // choose the pin for the LED
int powerPin = 3;               // choose the power pin (for PIR sensor)
int inputPin = 2;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
 
void setup() {
  pinMode(ledPin, OUTPUT);      // declare LED as output
  pinMode(powerPin, OUTPUT);      // declare LED as output
  pinMode(inputPin, INPUT);     // declare sensor as input
  digitalWrite(powerPin, HIGH);
  Serial.begin(9600);
}
 
void loop(){
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    digitalWrite(ledPin, HIGH);  // turn LED ON
    if (pirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected!");
      tone(speakerPin, talkFrequency);
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    digitalWrite(ledPin, LOW); // turn LED OFF
    if (pirState == HIGH){
      // we have just turned of
      Serial.println("Motion ended!");
      noTone(speakerPin);
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
}
