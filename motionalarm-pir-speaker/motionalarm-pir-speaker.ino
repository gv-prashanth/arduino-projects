const int pirInterruptPin = 2;//pin 2 only should be used
const int speakerPin = 4;
const int talkFrequency = 2000;//frequency in Hz
volatile boolean isMarkedForMotionDetection = false;

void setup() {
  Serial.begin (9600);
  attachInterrupt(digitalPinToInterrupt(pirInterruptPin), motionDetectedRoutine, RISING);
  //attachInterrupt(digitalPinToInterrupt(pirInterruptPin), motionAbsentRoutine, FALLING);
}

void loop() {
  if (isMarkedForMotionDetection) {
    Serial.println("Motion Detected");
    tone(speakerPin, talkFrequency, 1000);
    delay(1000);
    isMarkedForMotionDetection = false;
  }
}

void motionDetectedRoutine() {
  isMarkedForMotionDetection = true;
}
