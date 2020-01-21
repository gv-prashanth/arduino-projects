const int speakerPin = 4;

void setup() {
  Serial.begin (9600);
  pinMode(speakerPin, OUTPUT);
}

void loop() {
  for(int talkFrequency = 0; talkFrequency < 4000; talkFrequency = talkFrequency + 100){
   Serial.println(talkFrequency);
   tone(speakerPin, talkFrequency, 2000);
   delay(3000); 
  }
}
