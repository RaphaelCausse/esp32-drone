#ifndef ESP32_DRONE_PID_CONTROLLER_H
#define ESP32_DRONE_PID_CONTROLLER_H

#include <Arduino.h>

class PIDController
{
public:
    PIDController();
    ~PIDController();

    void init(float kp, float ki, float kd);
    float compute(float current, float target);
    void reset();

private:
    float clamp(float value, float min, float max);

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_integral = 0.0f;
    float m_prev_error = 0.0f;
    uint32_t m_last_time = 0;

private:
    static constexpr float INTEGRAL_WINDUP_VALUE = 400.0f;
};

#endif /* ESP32_DRONE_PID_CONTROLLER_H */
