#include <avr/sleep.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("booting complete");
}

void loop() {
  Serial.println("Going to sleep in 5 seconds");
  delay(5000);
  Going_To_Sleep();
  Serial.println("Sleep Completed");
  delay(5000);
}

void Going_To_Sleep(){
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();
}
