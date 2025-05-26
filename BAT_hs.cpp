#include "BAT_hs.h"

int8_t pin_adc;

static uint8_t mvToPercent(int32_t mvolts) 
{
  if(mvolts<3300)
    return 0;

  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }

  if(mvolts >4200) {
    return 120;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}

void BAT_init(int pin)
{
  pin_adc=pin;
  analogReadResolution(12);
  analogSetClockDiv(10);
}

uint16_t BAT_loop()
{
  int analogVolts = analogReadMilliVolts(pin_adc);
  return mvToPercent(analogVolts*23/9);
}









