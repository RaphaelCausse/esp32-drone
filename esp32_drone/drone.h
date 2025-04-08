#ifndef ESP32_DRONE_DRONE_H
#define ESP32_DRONE_DRONE_H

#include <Arduino.h>
#include <Wire.h>
#include "drone_state.h"
#include "sensor_mpu6050.h"
#include "sensor_vl53l0x.h"
#include "pid_controller.h"
// #include "flight_receiver.h"
#include "led_rgb.h"

#define DRONE_I2C_SDA (8) // GPIO pin for I2C SDA
#define DRONE_I2C_SCL (9) // GPIO pin for I2C SCL

#define LED_BLINK_INTERVAL_ERROR (500) // Blink interval in milliseconds for ERROR state
#define LED_BLINK_INTERVAL_AUTO (500)  // Blink interval in milliseconds for AUTO_TAKEOFF and AUTO_LANDING states

#define DELAY_IDLE (500)

#define PID_GAIN_P_ROLL (1.0f)  // Proportional gain for roll (reacts to current roll error)
#define PID_GAIN_I_ROLL (0.1)   // Integral gain for roll (accumulates past roll error)
#define PID_GAIN_D_ROLL (0.01f) // Derivative gain for roll (reacts to rate of change in roll error)

#define PID_GAIN_P_PITCH (1.0f)  // Proportional gain for pitch (reacts to current pitch error)
#define PID_GAIN_I_PITCH (0.1f)  // Integral gain for pitch (accumulates past pitch error)
#define PID_GAIN_D_PITCH (0.01f) // Derivative gain for pitch (reacts to rate of change in pitch error)

#define PID_GAIN_P_YAW (1.0f)  // Proportional gain for yaw (reacts to current yaw error)
#define PID_GAIN_I_YAW (0.01f) // Integral gain for yaw (accumulates past yaw error)
#define PID_GAIN_D_YAW (0.01f) // Derivative gain for yaw (reacts to rate of change in yaw error)

class Drone
{
public:
    Drone();
    ~Drone();

    void init();
    void update();

    void stabilize_drone();

private:
    const char *state_to_cstr(DroneState state);
    void change_state(DroneState new_state);

private:
    DroneState m_state;
    bool m_emergency_protocol_engaged = false;

    SensorMPU6050 m_imu; // Inertial Measurement Unit Sensor
    SensorVL53L0X m_tof; // Time Of Flight Distance Sensor

    PIDController m_pid_roll;  // PID controller for roll
    PIDController m_pid_pitch; // PID controller for pitch
    PIDController m_pid_yaw;   // PID controller for yaw

    // FlightReceiver m_flight_receiver;
};

#endif /* ESP32_DRONE_DRONE_H */
