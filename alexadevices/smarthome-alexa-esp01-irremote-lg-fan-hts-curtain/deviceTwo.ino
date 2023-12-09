// true : 0x1DED
// false : 0x15ED
boolean power_toggle_D = false;
// true : 0x1DE1
// false : 0x15E1
boolean speed1_toggle_D = false;
// true : 0x1DE2
// false : 0x15E2
boolean speed2_toggle_D = false;
// true : 0x1DE3
// false : 0x15E3
boolean speed3_toggle_D = false;
// true : 0x1DE4
// false : 0x15E4
boolean speed4_toggle_D = false;
// true : 0x1DE5
// false : 0x15E5
boolean speed5_toggle_D = false;
int previousSpeed = -1;
int MAX_TRIES = 10;
int DELAY_TIME = 50;

void Fan_Activate(unsigned int speed) {
  if (previousSpeed == -1) {
    togglePower();
    previousSpeed = speed;
  }
  if (previousSpeed != speed) {
    if (speed == 1) {
      toggleSpeed1();
    } else if (speed == 2) {
      toggleSpeed2();
    } else if (speed == 3) {
      toggleSpeed3();
    } else if (speed == 4) {
      toggleSpeed4();
    } else {
      toggleSpeed5();
    }
    previousSpeed = speed;
  }
}

void Fan_Power_Down() {
  togglePower();
  previousSpeed = -1;
}

void togglePower() {
  for (int i = 0; i < MAX_TRIES; i++) {
    if (power_toggle_D)
      irSend.sendRC5(0x1DED, 13);
    else
      irSend.sendRC5(0x15ED, 13);
    delay(DELAY_TIME);
  }
  power_toggle_D = !power_toggle_D;
}

void toggleSpeed1() {
  for (int i = 0; i < MAX_TRIES; i++) {
    if (speed1_toggle_D)
      irSend.sendRC5(0x1DE1, 13);
    else
      irSend.sendRC5(0x15E1, 13);
    delay(DELAY_TIME);
  }
  speed1_toggle_D = !speed1_toggle_D;
}

void toggleSpeed2() {
  for (int i = 0; i < MAX_TRIES; i++) {
    if (speed2_toggle_D)
      irSend.sendRC5(0x1DE3, 13); //Actually 2 is faster than 3 for our fan. Hence intechanged to solve via software
    else
      irSend.sendRC5(0x15E3, 13);
    delay(DELAY_TIME);
  }
  speed2_toggle_D = !speed2_toggle_D;
}

void toggleSpeed3() {
  for (int i = 0; i < MAX_TRIES; i++) {
    if (speed3_toggle_D)
      irSend.sendRC5(0x1DE2, 13); //Actually 2 is faster than 3 for our fan. Hence intechanged to solve via software
    else
      irSend.sendRC5(0x15E2, 13);
    delay(DELAY_TIME);
  }
  speed3_toggle_D = !speed3_toggle_D;
}

void toggleSpeed4() {
  for (int i = 0; i < MAX_TRIES; i++) {
    if (speed4_toggle_D)
      irSend.sendRC5(0x1DE4, 13);
    else
      irSend.sendRC5(0x15E4, 13);
    delay(DELAY_TIME);
  }
  speed4_toggle_D = !speed4_toggle_D;
}

void toggleSpeed5() {
  for (int i = 0; i < MAX_TRIES; i++) {
    if (speed5_toggle_D)
      irSend.sendRC5(0x1DE5, 13);
    else
      irSend.sendRC5(0x15E5, 13);
    delay(DELAY_TIME);
  }
  speed5_toggle_D = !speed5_toggle_D;
}
