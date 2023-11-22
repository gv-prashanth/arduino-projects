#include <Arduino.h>
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SNoDAC.h"

#include "alarmsound.h"

//First record audio file
//Convert to wav using https://cloudconvert.com/m4a-to-wav
//Compress wav using https://www.freeconvert.com/wav-compressor
//Generate hex using https://tomeko.net/online_tools/file_to_hex.php?lang=en
AudioGeneratorWAV *wav;
AudioFileSourcePROGMEM *file;
AudioOutputI2SNoDAC *out;

void audioStop() {
  if (wav != nullptr && wav->isRunning()) {
    wav->stop();
    Serial.println("Audio stopped");
  }
}

void audioPlay() {
  if (wav != nullptr && wav->isRunning()) {
    boolean loopVar = wav->loop();
    if (loopVar) {
      //still playing
    } else {
      //playing finished
    }
  } else {
    //trigger a new play
    file = new AudioFileSourcePROGMEM(alarmsound, sizeof(alarmsound));
    out = new AudioOutputI2SNoDAC();
    wav = new AudioGeneratorWAV();
    wav->begin(file, out);
    Serial.println("Audio triggered");
  }
}