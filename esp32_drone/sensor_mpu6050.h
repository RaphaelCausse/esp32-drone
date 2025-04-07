#ifndef ESP32_DRONE_SENSOR_MPU6050_H
#define ESP32_DRONE_SENSOR_MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "sensor_state.h"

#define MPU6050_INIT_RETRY (5)

class SensorMPU6050
{
public:
    SensorMPU6050();
    ~SensorMPU6050();

    bool init(TwoWire *wire = &Wire);
    void calibrate();

    SensorState state() const;

private:
    TwoWire *m_i2c;
    SensorState m_state;
    Adafruit_MPU6050 m_mpu6050;
};

#endif /* ESP32_DRONE_SENSOR_MPU6050_H */
