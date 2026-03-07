/************ TONE CONFIG ************/
const unsigned long TOGGLE_US = 500;   // 1 kHz tone

/************ PATTERN CONFIG ************/
// Durations in microseconds
const unsigned long pattern[] = {
  100000,   // ON  100ms
  100000,   // OFF 100ms
  300000,   // ON  300ms
  500000    // OFF 500ms
};
const int PATTERN_LEN = sizeof(pattern) / sizeof(pattern[0]);

/************ STATE ************/
bool toneEnabled = false;
bool pinState = false;
int patternIndex = 0;
bool shouldbeep = false;

unsigned long lastToggle = 0;

void stopAlarm() {
  shouldbeep = false;
  // Ensure we leave the output in a quiet, deterministic state.
  toneEnabled = false;
  pinState = false;
  digitalWrite(ALARM_PIN, LOW);
  Serial.println("Turning Off the alarm output");
}

void startAlarm() {
  //Put some pin to high
  shouldbeep = true;
  // Reset pattern/tone state so we start cleanly.
  patternIndex = 0;
  phaseStart = micros();
  lastToggle = phaseStart;
  toneEnabled = true;
  pinState = false;
  digitalWrite(ALARM_PIN, LOW);
  Serial.println("Turning On the alarm output");
}

void alarmLoop() {
  if(!shouldbeep) {
    // If we are not actively beeping, keep pin quiet.
    if (pinState) {
      pinState = false;
      digitalWrite(ALARM_PIN, LOW);
    }
    return;
  }
  unsigned long now = micros();

  // Handle pattern phase change
  if (now - phaseStart >= pattern[patternIndex]) {
    patternIndex = (patternIndex + 1) % PATTERN_LEN;
    phaseStart = now;
    toneEnabled = (patternIndex % 2 == 0);  // even = ON, odd = OFF
    digitalWrite(ALARM_PIN, LOW);
    pinState = false;
  }

  // Generate tone when enabled
  if (toneEnabled && now - lastToggle >= TOGGLE_US) {
    lastToggle = now;
    pinState = !pinState;
    digitalWrite(ALARM_PIN, pinState);
  }
}