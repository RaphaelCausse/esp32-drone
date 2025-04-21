#include "pid_controller.h"

static const char *TAG = "PID_CTRL";

PIDController::PIDController() {}

PIDController::~PIDController() {}

void PIDController::init(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_integral = 0.0f;
    m_prev_error = 0.0f;
    m_last_time = millis();
}

float PIDController::compute(float current, float target)
{
    float error = target - current;
    unsigned long now = millis();
    float dt = (now - m_last_time) / 1000.0f;
    m_last_time = now;

    if (dt <= 0.0f)
    {
        dt = 0.01f;
    }

    m_integral += (error + m_prev_error) * dt / 2;

    // Avoid Integral windup
    m_integral = clamp(m_integral, -INTEGRAL_WINDUP_VALUE, INTEGRAL_WINDUP_VALUE);

    float derivative = (error - m_prev_error) / dt;
    m_prev_error = error;

    float output = m_kp * error + m_ki * m_integral + m_kd * derivative;
    output = clamp(output, -INTEGRAL_WINDUP_VALUE, INTEGRAL_WINDUP_VALUE);

    return output;
}

void PIDController::reset()
{
    m_integral = 0.0f;
    m_prev_error = 0.0f;
}

float PIDController::clamp(float value, float min, float max) const
{
    if (value > max)
    {
        return max;
    }
    if (value < min)
    {
        return min;
    }
    return value;
}