#include <Adafruit_VL53L0X.h>

#ifndef ESP32_DRONE_VL53L0X_H
#define ESP32_DRONE_VL53L0X_H

struct vlx_data_t
{
  uint16_t old_distance; // en mm
  uint16_t new_distance; // en mm
  uint16_t max_distance; // en cm
  VL53L0X_RangingMeasurementData_t measure;
};

bool vl53l0x_init(void);

void vl53l0x_poll(void);

void vl53l0x_LED(void);

#endif /* ESP32_DRONE_VL53L0X_H */