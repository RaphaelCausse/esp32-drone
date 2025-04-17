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
    void stop_all();

    void spin_idle(uint32_t current_ms);

private:
    void write_us_to_pin(int channel, uint16_t us);
    uint16_t constrain_us(uint16_t us, uint16_t min, uint16_t max);

private:
    static constexpr uint16_t MIN_PULSE_WIDTH_US = 1000;
    static constexpr uint16_t MAX_PULSE_WIDTH_US = 2000;
    static constexpr int PWM_FREQ = 50;       // 50Hz standard ESC
    static constexpr int PWM_RESOLUTION = 12; // 12-bit for microsecond precision

    static constexpr int DELAY_INIT_ESC = 3000; // Delay to let ESC initialize

    static constexpr int SPIN_ARMED_PERCENTAGE = 12; // Percentage of spin for armed motors
    static constexpr int SPIN_IDLE_PERCENTAGE = 18;  // Percentage of spin for idle motors

    // Channel assignment for ESP32
    static constexpr int MOTOR_1_ID = 1; // Front right (CCW)
    static constexpr int MOTOR_2_ID = 2; // Rear right (CW)
    static constexpr int MOTOR_3_ID = 3; // Rear left (CCW)
    static constexpr int MOTOR_4_ID = 4; // Front left (CW)

    // GPIO pin assignment
    static constexpr int MOTOR_1_PIN = 36;
    static constexpr int MOTOR_2_PIN = 35;
    static constexpr int MOTOR_3_PIN = 37;
    static constexpr int MOTOR_4_PIN = 38;
};

#endif /* ESP32_DRONE_MOTORS_CONTROLLER_H */