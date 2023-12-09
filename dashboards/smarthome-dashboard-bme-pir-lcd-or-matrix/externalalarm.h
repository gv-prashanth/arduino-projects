void stopAlarm() {
  //Put some pin to low
  digitalWrite(ALARM_PIN, LOW);
  Serial.println("Turning Off the alarm output");
}

void startAlarm() {
  //Put some pin to high
  digitalWrite(ALARM_PIN, HIGH);
  Serial.println("Turning On the alarm output");
}

void alarmLoop() {
  //Do nothing here
}