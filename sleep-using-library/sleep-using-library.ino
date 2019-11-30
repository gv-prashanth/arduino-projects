#include <avr/sleep.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("booting complete");
}

void loop() {
  Serial.println("Loop started");
  Serial.println("Going to sleep");
  Going_To_Sleep();
  Serial.println("Sleep Completed");
}

void Going_To_Sleep(){
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();
}
