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

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_integral = 0.0f;
    float m_prev_error = 0.0f;
    unsigned long m_last_time = 0;
};

#endif /* ESP32_DRONE_PID_CONTROLLER_H */
