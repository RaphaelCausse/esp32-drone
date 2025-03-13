// Function to turn on/off the RGB led in different color

#include "ledRGB.h"
#include <Arduino.h>

void ledRED(bool on){
  #ifdef RGB_BUILTIN
    if(on){
      rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
    }else{
      rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    }
  #endif
}

void ledGREEN(bool on){
  #ifdef RGB_BUILTIN
    if(on){
      rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
    }else{
      rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    }
  #endif
}

void ledBLUE(bool on){
  #ifdef RGB_BUILTIN
    if(on){
      rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
    }else{
      rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    }
  #endif
}

void ledPURPLE(bool on){
  #ifdef RGB_BUILTIN
    if(on){
      rgbLedWrite(RGB_BUILTIN, 127, 0, 127);
    }else{
      rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    }
  #endif
}