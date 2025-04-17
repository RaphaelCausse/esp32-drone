#ifndef ESP32_DRONE_SENSOR_MPU6050_H
#define ESP32_DRONE_SENSOR_MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "sensor_state.h"

class SensorMPU6050
{
public:
    SensorMPU6050();
    ~SensorMPU6050();

    bool init(TwoWire *wire = &Wire);
    bool calibrate();
    void update();
    bool check();
    bool read_gyro();
    bool read_accel();

    SensorState state() const;
    bool is_active() const;
    float roll() const;
    float pitch() const;
    float yaw() const;

private:
    TwoWire *m_i2c = nullptr;
    SensorState m_state;
    Adafruit_MPU6050 m_mpu6050;

    float m_roll_rate;         // Roll rate, around X axis (rad/s)
    float m_roll_calibration;  // Roll rate calibration value
    float m_pitch_rate;        // Pitch rate, around Y axis (rad/s)
    float m_pitch_calibration; // Pitch rate calibration value
    float m_yaw_rate;          // Yaw rate, around Z axis (rad/s)
    float m_yaw_calibration;   // Yaw rate calibration value

    float m_accel_x; // Acceleration on X axis (m/s^2)
    float m_accel_y; // Acceleration on Y axis (m/s^2)
    float m_accel_z; // Acceleration on Z axis (m/s^2)

private:
    static constexpr int MPU6050_INIT_RETRY = 5; // Number of retry for sensor initialization
};

#endif /* ESP32_DRONE_SENSOR_MPU6050_H */
