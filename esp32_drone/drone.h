#ifndef ESP32_DRONE_DRONE_H
#define ESP32_DRONE_DRONE_H

#include <Arduino.h>
#include <Wire.h>
#include "drone_state.h"
#include "sensor_mpu6050.h"
#include "sensor_vl53l0x.h"
// #include "flight_receiver.h"
#include "motors_controller.h"
#include "pid_controller.h"
#include "led_rgb.h"

class Drone
{
public:
    Drone();
    ~Drone();

    void init();
    void update();

private:
    void handle_state_calibrating();
    void handle_state_idle();
    void handle_state_disarmed();
    void handle_state_armed();
    void handle_state_auto_takeoff();
    void handle_state_flying();
    void handle_state_auto_landing();
    void handle_state_emergency_landing();
    void handle_state_error();

    bool check_sensors();
    void stabilize_drone();

    const char *state_to_cstr(DroneState state);
    void change_state(DroneState new_state);

private:
    DroneState m_state;
    bool m_emergency_protocol_engaged = false;
    uint32_t m_last_time_sensors_checked = 0;

    SensorMPU6050 m_imu; // Inertial Measurement Unit Sensor
    SensorVL53L0X m_tof; // Time Of Flight Distance Sensor

    // FlightReceiver m_flight_receiver;

    MotorsController m_motors; // Motors controller, PWM

    PIDController m_pid_roll;  // PID controller for roll
    PIDController m_pid_pitch; // PID controller for pitch
    PIDController m_pid_yaw;   // PID controller for yaw

private:
    static constexpr uint8_t DRONE_I2C_SDA = 8; // GPIO pin for I2C SDA
    static constexpr uint8_t DRONE_I2C_SCL = 9; // GPIO pin for I2C SCL

    static constexpr uint16_t LED_BLINK_INTERVAL_ERROR = 500; // Blink interval in milliseconds for ERROR state
    static constexpr uint16_t LED_BLINK_INTERVAL_AUTO = 500;  // Blink interval in milliseconds for AUTO_TAKEOFF and AUTO_LANDING states

    static constexpr uint16_t DELAY_MS_IDLE = 500;
    static constexpr uint16_t DELAY_MS_CHECK_SENSORS = 500; // Delay in milliseconds for sensors check

    static constexpr float PID_GAIN_P_ROLL = 1.0f;  // Proportional gain for roll (reacts to current roll error)
    static constexpr float PID_GAIN_I_ROLL = 0.1f;  // Integral gain for roll (accumulates past roll error)
    static constexpr float PID_GAIN_D_ROLL = 0.01f; // Derivative gain for roll (reacts to rate of change in roll error)

    static constexpr float PID_GAIN_P_PITCH = 1.0f;  // Proportional gain for pitch (reacts to current pitch error)
    static constexpr float PID_GAIN_I_PITCH = 0.1f;  // Integral gain for pitch (accumulates past pitch error)
    static constexpr float PID_GAIN_D_PITCH = 0.01f; // Derivative gain for pitch (reacts to rate of change in pitch error)

    static constexpr float PID_GAIN_P_YAW = 1.0f;  // Proportional gain for yaw (reacts to current yaw error)
    static constexpr float PID_GAIN_I_YAW = 0.1f;  // Integral gain for yaw (accumulates past yaw error)
    static constexpr float PID_GAIN_D_YAW = 0.01f; // Derivative gain for yaw (reacts to rate of change in yaw error)
};

#endif /* ESP32_DRONE_DRONE_H */
