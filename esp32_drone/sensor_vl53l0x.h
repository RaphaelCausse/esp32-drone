#ifndef ESP32_DRONE_SENSOR_VL53L0X_H
#define ESP32_DRONE_SENSOR_VL53L0X_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "sensor_state.h"

class SensorVL53L0X
{
public:
    SensorVL53L0X();
    ~SensorVL53L0X();

    bool init(TwoWire *wire = &Wire);
    void update();
    bool check();
    bool read_distance();

    SensorState state() const;
    bool is_active() const;
    float distance_cm() const;

private:
    TwoWire *m_i2c = nullptr;
    SensorState m_state;
    Adafruit_VL53L0X m_vl53l0x;

    float m_distance_cm; // Distance (cm)

private:
    static constexpr int VL53L0X_INIT_RETRY = 5; // Number of retry for sensor initialization
};

#endif /* ESP32_DRONE_SENSOR_VL53L0X_H */
