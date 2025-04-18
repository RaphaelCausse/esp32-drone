#ifndef ESP32_DRONE_MOTORS_CONTROLLER_H
#define ESP32_DRONE_MOTORS_CONTROLLER_H

#include <Arduino.h>

class MotorsController
{
public:
    MotorsController();
    ~MotorsController();

    bool init();
    bool setup_motor(int pin);
    void write_pwm(uint8_t motor_id, uint16_t us);
    void init_sequence_esc();
    void stop_motors(float duration_seconds);
    void spin_motors_armed();
    void compute_motor_inputs(float input_throttle, float input_roll, float input_pitch, float input_yaw);
    void send_motor_inputs();

private:
    void write_us_to_pin(int channel, uint16_t us);
    uint16_t constrain_us(uint16_t us, uint16_t min, uint16_t max) const;

private:
    uint16_t m_motor_last_pwm[4]; // Stores the last PWM value for each motor

    float m_input_motor1; // Front right motor (CCW)
    float m_input_motor2; // Rear right motor (CW)
    float m_input_motor3; // Rear left motor (CCW)
    float m_input_motor4; // Front left motor (CW)

public:
    static constexpr uint16_t PULSE_WIDTH_US_MAX = 2000; // Maximum pulse width in microseconds
    static constexpr uint16_t PULSE_WIDTH_US_MIN = 1000; // Minimum pulse width in microseconds
    static constexpr int PWM_FREQ = 50;                  // 50Hz standard ESC
    static constexpr int PWM_RESOLUTION = 12;            // 12-bit for microsecond precision

    static constexpr int DELAY_INIT_ESC = 3000; // Delay to let ESC initialize

    static constexpr int PULSE_WIDTH_US_SPIN_ARMED = 1120; // Pulse to spin motor when armed at 12%
    static constexpr int PULSE_WIDTH_US_SPIN_IDLE = 1180;  // Pulse to spin motor when idle at 18%
    static constexpr int PULSE_WIDTH_US_SPIN_HALF = 1500;  // Pulse to spin motor at 50%

    // Channel assignment for ESP32
    static constexpr int MOTOR_1_ID = 1;
    static constexpr int MOTOR_2_ID = 2;
    static constexpr int MOTOR_3_ID = 3;
    static constexpr int MOTOR_4_ID = 4;

    // GPIO pin assignment
    static constexpr int MOTOR_1_PIN = 36;
    static constexpr int MOTOR_2_PIN = 35;
    static constexpr int MOTOR_3_PIN = 37;
    static constexpr int MOTOR_4_PIN = 38;
};

#endif /* ESP32_DRONE_MOTORS_CONTROLLER_H */