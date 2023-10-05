void toggleCurtain() {
  if (!curtainIsOpen)
    openCurtain();
  else
    closeCurtain();
  curtainIsOpen = !curtainIsOpen;
}

void openCurtain() {
  Serial.println("CURTAIN OPENING");
  // Put the motor in forward:
  delay(500);                 // delay for Controller startup
  digitalWrite(dir, HIGH);    // Turn MOTOR FORWARD / OPEN CURTAIN
  digitalWrite(Enable, LOW);  // Enable Stepper Motor

  for (double i = 0; i < 2300; i++)  //6600 steps in one direction
  {
    digitalWrite(step, HIGH);
    delay(1);
    digitalWrite(step, LOW);
    delay(1);
  }
  digitalWrite(Enable, HIGH);  //Disable Stepper Motor
}

void closeCurtain() {
  Serial.println("CURTAIN CLOSING");
  // Put the motor in reverse:
  delay(500);                 // delay for Controller startup
  digitalWrite(dir, LOW);     // Turn MOTOR REVBERSE / CLOSE CURTAIN
  digitalWrite(Enable, LOW);  // Enable Stepper Motor

  for (double i = 0; i < 2300; i++)  //6400 steps in one direction
  {
    digitalWrite(step, HIGH);
    delay(1);
    //delayMicroseconds(1000);
    digitalWrite(step, LOW);
    delay(1);
    //delayMicroseconds(1000);
  }
  digitalWrite(Enable, HIGH);  //Disable Stepper Motor
}