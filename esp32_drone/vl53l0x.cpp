#include <Adafruit_VL53L0X.h>
#include "vl53l0x.h"
#include "logger.h"
#include "ledRGB.h"

static const char *TAG = "vl53l0x";

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

uint8_t max_distance = 4;


bool vl53l0x_init(void)
{
  logger.info(TAG, "Initializing vl53l0x device");
  if (!lox.begin()) {
    Serial.println(F("Echec during initialize of VL53L0X"));
    return false;
  }
  return true;
}

void vl53l0x_poll(void)
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if(measure.RangeStatus != max_distance){
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  }else{
    Serial.println("Trop loin !");
  }
  // delay(100);
}

void vl53l0x_LED(uint16_t distance)
{
  if (distance < 200){

    ledRED(true);
  }
  else{
    ledRED(false);
  }
}