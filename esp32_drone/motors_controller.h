#ifndef ESP32_DRONE_MOTORS_CONTROLLER_H
#define ESP32_DRONE_MOTORS_CONTROLLER_H

#include <Arduino.h>

class MotorsController
{
public:
    MotorsController();
    ~MotorsController();

    void init();
    void write_pwm(uint8_t motor_id, uint16_t us);
    void stop_all();

private:
    void setup_motor(int channel, int pin);
    void write_us_to_channel(int channel, uint16_t us);

private:
    static constexpr uint16_t MIN_PULSE_WIDTH_US = 1000;
    static constexpr uint16_t MAX_PULSE_WIDTH_US = 2000;
    static constexpr int PWM_FREQ = 50;       // 50Hz standard ESC
    static constexpr int PWM_RESOLUTION = 16; // 16-bit for microsecond precision

    // Channel assignment for ESP32
    static constexpr int MOTOR_1_CH = 0;
    static constexpr int MOTOR_2_CH = 1;
    static constexpr int MOTOR_3_CH = 2;
    static constexpr int MOTOR_4_CH = 3;

    // GPIO pin assignment
    static constexpr int MOTOR_1_PIN = 36; // Front right (CCW)
    static constexpr int MOTOR_2_PIN = 35; // Rear right (CW)
    static constexpr int MOTOR_3_PIN = 37; // Rear left (CCW)
    static constexpr int MOTOR_4_PIN = 38; // Front left (CW)
};

#endif /* ESP32_DRONE_MOTORS_CONTROLLER_H */