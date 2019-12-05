#include <DigitalBase.h>

//Pin Configuration
const int leftWheelForwardPin = 5;//5
const int leftWheelBackwardPin = 9;//9
const int rightWheelForwardPin = 11;//10
const int rightWheelBackwardPin = 10;//11
const int baseEnablePin = 6;

void setup() {
  Serial.begin (9600);
  delay(5000);
}

void loop() {
  //left
  rotateLeft();
  delay(5000);
  stopAllMotion();
  delay(1000);

  //right
  rotateRight();
  delay(5000);
  stopAllMotion();
  delay(1000);

  //forward
  goForward();
  delay(5000);
  stopAllMotion();
  delay(1000);

  //backward
  moveBackward();
  delay(5000);
  stopAllMotion();
  delay(1000);
}

void stopAllMotion() {
  digitalWrite(baseEnablePin, LOW);
  digitalWrite(leftWheelForwardPin, LOW);
  digitalWrite(leftWheelBackwardPin, LOW);
  digitalWrite(rightWheelForwardPin, LOW);
  digitalWrite(rightWheelBackwardPin, LOW);
}

void moveBackward() {
  digitalWrite(baseEnablePin, HIGH);
  digitalWrite(leftWheelForwardPin, LOW);
  digitalWrite(leftWheelBackwardPin, HIGH);
  digitalWrite(rightWheelForwardPin, LOW);
  digitalWrite(rightWheelBackwardPin, HIGH);
}


void goForward() {
  digitalWrite(baseEnablePin, HIGH);
  digitalWrite(leftWheelForwardPin, HIGH);
  digitalWrite(leftWheelBackwardPin, LOW);
  digitalWrite(rightWheelForwardPin, HIGH);
  digitalWrite(rightWheelBackwardPin, LOW);
}


void rotateRight() {
  digitalWrite(baseEnablePin, HIGH);
  digitalWrite(leftWheelForwardPin, HIGH);
  digitalWrite(leftWheelBackwardPin, LOW);
  digitalWrite(rightWheelForwardPin, LOW);
  digitalWrite(rightWheelBackwardPin, HIGH);
}


void rotateLeft() {
  digitalWrite(baseEnablePin, HIGH);
  digitalWrite(leftWheelForwardPin, LOW);
  digitalWrite(leftWheelBackwardPin, HIGH);
  digitalWrite(rightWheelForwardPin, HIGH);
  digitalWrite(rightWheelBackwardPin, LOW);
}
