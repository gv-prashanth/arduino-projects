const int pirInterruptPin = 2;//pin 2 only should be used
const int speakerPin = 4;
const int talkFrequency = 2000;//frequency in Hz

void setup() {
  Serial.begin (9600);
  pinMode(pirInterruptPin, INPUT);     // declare sensor as input
}

void loop() {
  int val = digitalRead(pirInterruptPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    tone(speakerPin, talkFrequency, 5000);
    delay(5000);
  }
}
