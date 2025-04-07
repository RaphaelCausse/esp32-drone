#ifndef ESP32_DRONE_DRONE_H
#define ESP32_DRONE_DRONE_H

#include <Arduino.h>
#include <Wire.h>
#include "drone_state.h"
#include "sensor_mpu6050.h"
#include "sensor_vl53l0x.h"
#include "led_rgb.h"

#define DRONE_I2C_SDA (8) // GPIO pin for I2C SDA
#define DRONE_I2C_SCL (9) // GPIO pin for I2C SCL

#define LED_BLINK_INTERVAL_IDLE (1000)     // Blink interval in milliseconds for IDLE state
#define LED_BLINK_INTERVAL_DISARMED (1000) // Blink interval in milliseconds for DISARMED state
#define LED_BLINK_INTERVAL_ARMED (1000)    // Blink interval in milliseconds for ARMED state
#define LED_BLINK_INTERVAL_ERROR (500)     // Blink interval in milliseconds for ERROR state

class Drone
{
public:
    Drone();
    ~Drone();

    void init();
    void update();

private:
    const char *state_to_str(DroneState state);
    void change_state(DroneState new_state);

private:
    DroneState m_state;
    SensorMPU6050 m_imu; // Inertial Measurement Unit Sensor
    SensorVL53L0X m_tof; // Time Of Flight Distance Sensor
};

#endif /* ESP32_DRONE_DRONE_H */
