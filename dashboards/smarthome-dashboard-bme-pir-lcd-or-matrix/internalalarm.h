#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SNoDAC.h"
#include "alarmaudio.h"
//#include "welcomeaudio.h"

//First record audio file
//Convert to wav using https://cloudconvert.com
//Compress wav using https://www.freeconvert.com/wav-compressor
//Generate hex using https://tomeko.net/online_tools/file_to_hex.php?lang=en
AudioGeneratorWAV* wav;
AudioFileSourcePROGMEM* file;
AudioOutputI2SNoDAC* out;

volatile boolean playAudio;
const unsigned char* globalAudioData = nullptr;
size_t globalDataSize = 0;

void stopAudio() {
  if (wav != nullptr) {
    wav->stop();
    playAudio = false;
    Serial.println("Audio stopped");
  }
}

void startAudio(const unsigned char audioData[], size_t dataSize) {
  stopAudio();
  delete file;
  delete out;
  delete wav;

  file = new AudioFileSourcePROGMEM(audioData, dataSize);
  out = new AudioOutputI2SNoDAC();
  wav = new AudioGeneratorWAV();
  playAudio = true;
  wav->begin(file, out);
  // Save audio data and size to global variables
  globalAudioData = audioData;
  globalDataSize = dataSize;

  Serial.println("Audio triggered");
}

void audioLoopSection() {
  if (wav != nullptr) {
    if (wav->isRunning()) {
      if (!wav->loop()) wav->stop();
    } else {
      if (playAudio && globalAudioData != nullptr) {
        Serial.println("This Audio session is done. Starting new session");
        startAudio(globalAudioData, globalDataSize);
      }
    }
  }
}

void stopAlarm() {
  stopAudio();
}

void startAlarm() {
  startAudio(alarmAudio, sizeof(alarmAudio));
}

void alarmLoop() {
  audioLoopSection();
}