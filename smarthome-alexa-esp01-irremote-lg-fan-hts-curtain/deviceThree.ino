const int MIN_VOLUME = 0;
const int MAX_VOLUME = 50;
int CURRENT_VOLUME = 5;
int INCREASE_STEP = 10;
int WAIT_TIME = 50;

void Decoder_ActivateArc(unsigned int volume) {
  irSend.sendNEC(0xFF10EF, 32);
  if (volume > CURRENT_VOLUME) {
    increaseVolume();
    volume++;
  } else if (volume < CURRENT_VOLUME) {
    decreaseVolume();
    volume--;
  }
}

void Decoder_ActivateOptical(unsigned int volume) {
  irSend.sendNEC(0xFF5AA5, 32);
  if (volume > CURRENT_VOLUME) {
    increaseVolume();
    volume++;
  } else if (volume < CURRENT_VOLUME) {
    decreaseVolume();
    volume--;
  }
}

void Decoder_Power_Down() {
  toggleDecoderPower();
}

void toggleDecoderPower() {
  irSend.sendNEC(0xFFA25D, 32);
}

void increaseVolume() {
  for (int i = 0; i < INCREASE_STEP; i++) {
    irSend.sendNEC(0xFF02FD, 32);
    delay(WAIT_TIME);
  }
}

void decreaseVolume() {
  for (int i = 0; i < INCREASE_STEP; i++) {
    irSend.sendNEC(0xFF9867, 32);
    delay(WAIT_TIME);
  }
}


int convertValueToSound(int value) {
  unsigned int volume = (100.0 / 255.0) * (value);
  if (volume <= MIN_VOLUME)
    volume = MIN_VOLUME;
  if (volume >= MAX_VOLUME)
    volume = MAX_VOLUME;
  return volume;
}
