/*
  DeepSleep.h - Library for base movement.
  Created by Vader, December 07, 2019.
  Released into the public domain.
*/

#include "Arduino.h"
#include "DeepSleep.h"

DeepSleep::DeepSleep()
{
  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1 << 6); //enable interrupt mode

  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
}

void DeepSleep::sleepForEightSecondsUnlessInterrupted(){
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application
  ADCSRA &= ~(1 << 7);

  //Begin the actual sleep
  __asm__  __volatile__("sleep");//in line assembler to go to sleep

  //Enable ADC - don't forget to flip back after waking up if using ADC in your application
  ADCSRA |= (1 << 7);
}