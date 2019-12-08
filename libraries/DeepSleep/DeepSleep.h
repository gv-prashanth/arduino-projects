/*
  DeepSleep.h - Library for base movement.
  Created by Vader, December 07, 2019.
  Released into the public domain.
*/

#ifndef DeepSleep_h
#define DeepSleep_h

#include "Arduino.h"

class DeepSleep
{
  public:
    DeepSleep();
    void sleepForEightSecondsUnlessInterrupted();
};

#endif