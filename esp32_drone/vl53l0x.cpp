#include "logger.h"
#include "vl53l0x.h"
#include "ledRGB.h"

static const char *TAG = "vl53l0x";

Adafruit_VL53L0X vlx = Adafruit_VL53L0X();

vlx_data_t vlx_data;

bool vl53l0x_init(void)
{
  logger.info(TAG, "Initializing vl53l0x device");
  if (!vlx.begin()) {
    logger.error(TAG, "Failed to initialize of %s", TAG);
    return false;
  }
  vlx_data.max_distance = 4;
  return true;
}

void vl53l0x_poll(void)
{
  //measure
  vlx.rangingTest(&vlx_data.measure, false);
  vlx_data.old_distance = vlx_data.new_distance;
  //chack if measure is same as the max distance
  if(vlx_data.measure.RangeStatus != vlx_data.max_distance){
    vlx_data.new_distance = vlx_data.measure.RangeMilliMeter;
  }else{
    vlx_data.new_distance = vlx_data.max_distance*100; //passage en millimetre
  }
  logger.info(TAG, "Distance (mm): %u", vlx_data.new_distance);
}

void vl53l0x_LED(void)
{
  if(vlx_data.new_distance < 100){
    if(vlx_data.old_distance >= 100){
      ledRED(true);
    }
  }else if(vlx_data.new_distance >= 100 && vlx_data.new_distance <=300){
    if(vlx_data.old_distance < 100 || vlx_data.old_distance >300){
      ledORANGE(true);
    }
  }else{
    if(vlx_data.old_distance <= 300){
      ledGREEN(true);
    }
  }
}