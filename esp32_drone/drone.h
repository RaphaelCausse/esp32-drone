#ifndef ESP32_DRONE_DRONE_H
#define ESP32_DRONE_DRONE_H

#include <Arduino.h>
#include <Wire.h>
#include "drone_state.h"
#include "sensor_mpu6050.h"
#include "sensor_vl53l0x.h"
#include "flight_receiver.h"
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
    void handle_state_calibrating(uint32_t current_ms);
    void handle_state_disarmed(uint32_t current_ms);
    void handle_state_armed(uint32_t current_ms);
    void handle_state_auto_takeoff(uint32_t current_ms);
    void handle_state_flying(uint32_t current_ms);
    void handle_state_auto_landing(uint32_t current_ms);
    void handle_state_emergency_landing(uint32_t current_ms);
    void handle_state_error(uint32_t current_ms);
    bool check_sensors();
    const char *state_to_cstr(DroneState state);
    void change_state(DroneState new_state);

private:
    DroneState m_state;
    bool m_emergency_protocol_engaged = false;
    uint32_t m_last_sensors_checked_ms = 0;
    uint32_t m_last_pwm_update_ms = 0;

    SensorMPU6050 m_imu; // Inertial Measurement Unit Sensor
    SensorVL53L0X m_tof; // Time Of Flight Distance Sensor

    FlightReceiver m_flight_receiver; // Flight receiver using ESPNOW

    MotorsController m_motors; // Motors controller, PWM

    PIDController m_pid_roll;  // PID controller for roll
    PIDController m_pid_pitch; // PID controller for pitch
    PIDController m_pid_yaw;   // PID controller for yaw

    float m_current_throttle;
    float m_current_altitude;

public:
    static constexpr uint8_t DRONE_I2C_SDA = 8; // GPIO pin for I2C SDA
    static constexpr uint8_t DRONE_I2C_SCL = 9; // GPIO pin for I2C SCL

    static constexpr uint32_t LOOP_DURATION_MS = 50;

    static constexpr uint16_t LED_BLINK_INTERVAL_NORMAL = 500; // Normal speend for blink interval, in milliseconds
    static constexpr uint16_t LED_BLINK_INTERVAL_FAST = 250;   // Fast blink interval, in milliseconds

    static constexpr uint16_t DELAY_MS_CHECK_SENSORS = 500; // Delay in milliseconds for sensors check
    static constexpr uint16_t DELAY_MS_SEND_PWM = 20;       // Delay in milliseconds for sending PWM signals to motors

    static constexpr float DURATION_S_STOP_MOTOR_NORMAL = 3.0f; // Normal duration in seconds to stop motors
    static constexpr float DURATION_S_STOP_MOTOR_FAST = 1.0f;   // Fast duration in seconds to stop motors

    static constexpr float PID_GAIN_P_ROLL = 0.6f;  // Proportional gain for roll (reacts to current roll error)
    static constexpr float PID_GAIN_I_ROLL = 3.5f;  // Integral gain for roll (accumulates past roll error)
    static constexpr float PID_GAIN_D_ROLL = 0.03f; // Derivative gain for roll (reacts to rate of change in roll error)

    static constexpr float PID_GAIN_P_PITCH = 0.6f;  // Proportional gain for pitch (reacts to current pitch error)
    static constexpr float PID_GAIN_I_PITCH = 3.5f;  // Integral gain for pitch (accumulates past pitch error)
    static constexpr float PID_GAIN_D_PITCH = 0.03f; // Derivative gain for pitch (reacts to rate of change in pitch error)

    static constexpr float PID_GAIN_P_YAW = 2.0f;  // Proportional gain for yaw (reacts to current yaw error)
    static constexpr float PID_GAIN_I_YAW = 12.0f; // Integral gain for yaw (accumulates past yaw error)
    static constexpr float PID_GAIN_D_YAW = 0.0f;  // Derivative gain for yaw (reacts to rate of change in yaw error)

    static constexpr float TARGET_ALTITUDE_CM = 34.0f;
    static constexpr float LANDING_THRESHOLD_CM = 7.5f;

    static constexpr float RATE_PWM_US_CLIMB = 5.0f;
    static constexpr float RATE_PWM_US_DESCEND = 5.0f;
};

#endif /* ESP32_DRONE_DRONE_H */
