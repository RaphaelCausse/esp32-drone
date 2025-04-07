#ifndef ESP32_DRONE_SENSOR_VL53L0X_H
#define ESP32_DRONE_SENSOR_VL53L0X_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "sensor_state.h"

#define VL53L0X_INIT_RETRY (5)

class SensorVL53L0X
{
public:
    SensorVL53L0X();
    ~SensorVL53L0X();

    bool init(TwoWire *wire = &Wire);

    SensorState state() const;

private:
    TwoWire *m_i2c;
    SensorState m_state;
    Adafruit_VL53L0X m_vl53l0x;
};

#endif /* ESP32_DRONE_SENSOR_VL53L0X_H */
