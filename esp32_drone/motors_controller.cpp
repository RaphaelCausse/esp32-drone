#include "esp32-hal-ledc.h"
#include "motors_controller.h"
#include "logger.h"

static const char *TAG = "MOTORS";

MotorsController::MotorsController()
{
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);
}

MotorsController::~MotorsController() {}

bool MotorsController::init()
{
    if (!setup_motor(MOTOR_1_PIN))
    {
        logger.error(TAG, "Failed to setup motor 1");
        return false;
    }
    if (!setup_motor(MOTOR_2_PIN))
    {
        logger.error(TAG, "Failed to setup motor 2");
        return false;
    }
    if (!setup_motor(MOTOR_3_PIN))
    {
        logger.error(TAG, "Failed to setup motor 3");
        return false;
    }
    if (!setup_motor(MOTOR_4_PIN))
    {
        logger.error(TAG, "Failed to setup motor 4");
        return false;
    }

    // Initialization sequence for ESC
    init_sequence_esc();

    logger.info(TAG, "Motors initialized");

    return true;
}

bool MotorsController::setup_motor(int pin)
{
    return ledcAttach(pin, PWM_FREQ, PWM_RESOLUTION);
}

void MotorsController::write_pwm(uint8_t motor_id, uint16_t us)
{
    us = constrain_us(us, PULSE_WIDTH_US_MIN, PULSE_WIDTH_US_MAX);

    int motor_index = motor_id - 1;
    if (motor_index < 0 || motor_index >= 4)
    {
        logger.warning(TAG, "Invalid motor ID: %d", motor_id);
        return;
    }

    m_motor_last_pwm[motor_index] = us; // store the last PWM for the motor
    int pin = 0;

    switch (motor_id)
    {
    case MOTOR_1_ID:
        pin = MOTOR_1_PIN;
        break;
    case MOTOR_2_ID:
        pin = MOTOR_2_PIN;
        break;
    case MOTOR_3_ID:
        pin = MOTOR_3_PIN;
        break;
    case MOTOR_4_ID:
        pin = MOTOR_4_PIN;
        break;
    }

    // Send PWM signal
    write_us_to_pin(pin, us);
}

void MotorsController::init_sequence_esc()
{
    logger.info(TAG, "Init ESC sequence, sending PWM signals (1000µs)");
    write_pwm(MOTOR_1_ID, PULSE_WIDTH_US_MIN);
    write_pwm(MOTOR_2_ID, PULSE_WIDTH_US_MIN);
    write_pwm(MOTOR_3_ID, PULSE_WIDTH_US_MIN);
    write_pwm(MOTOR_4_ID, PULSE_WIDTH_US_MIN);
    delay(DELAY_INIT_ESC);
}

void MotorsController::stop_motors(float duration_seconds)
{
    const uint16_t update_interval_ms = 50;
    const uint16_t total_steps = duration_seconds * 1000 / update_interval_ms;

    // Save PWM start value of each motor
    uint16_t start_pwm[4];
    for (int i = 0; i < 4; ++i)
    {
        start_pwm[i] = m_motor_last_pwm[i];
    }

    for (uint16_t step = 0; step <= total_steps; ++step)
    {
        float progress = static_cast<float>(step) / total_steps;

        for (int i = 0; i < 4; ++i)
        {
            // Linear interpolation between start_pwm[i] and PULSE_WIDTH_US_MIN
            uint16_t pwm = static_cast<uint16_t>(start_pwm[i] * (1.0f - progress) + PULSE_WIDTH_US_MIN * progress);
            if (pwm < PULSE_WIDTH_US_MIN)
            {
                pwm = PULSE_WIDTH_US_MIN;
            }

            write_pwm(i + 1, pwm);
            m_motor_last_pwm[i] = pwm;
        }
        delay(update_interval_ms);
    }

    // Security, force motor to stop
    for (int i = 0; i < 4; ++i)
    {
        write_pwm(i + 1, PULSE_WIDTH_US_MIN);
        m_motor_last_pwm[i] = PULSE_WIDTH_US_MIN;
    }
}

void MotorsController::spin_motors_armed()
{
    write_pwm(MOTOR_1_ID, PULSE_WIDTH_US_SPIN_ARMED);
    write_pwm(MOTOR_2_ID, PULSE_WIDTH_US_SPIN_ARMED);
    write_pwm(MOTOR_3_ID, PULSE_WIDTH_US_SPIN_ARMED);
    write_pwm(MOTOR_4_ID, PULSE_WIDTH_US_SPIN_ARMED);
}

void MotorsController::compute_motor_inputs(float input_throttle, float input_roll, float input_pitch, float input_yaw)
{
    m_input_motor1 = input_throttle - input_roll - input_pitch - input_yaw;
    m_input_motor2 = input_throttle - input_roll + input_pitch + input_yaw;
    m_input_motor2 = input_throttle + input_roll + input_pitch - input_yaw;
    m_input_motor2 = input_throttle + input_roll - input_pitch + input_yaw;

    // Limit maximum pulse
    if (m_input_motor1 > (float)PULSE_WIDTH_US_MAX)
    {
        m_input_motor1--;
    }
    if (m_input_motor2 > (float)PULSE_WIDTH_US_MAX)
    {
        m_input_motor2--;
    }
    if (m_input_motor3 > (float)PULSE_WIDTH_US_MAX)
    {
        m_input_motor3--;
    }
    if (m_input_motor4 > (float)PULSE_WIDTH_US_MAX)
    {
        m_input_motor4--;
    }

    // Ensure minimum pulse
    if (m_input_motor1 < PULSE_WIDTH_US_MIN)
    {
        m_input_motor1 = (float)PULSE_WIDTH_US_MIN;
    }
    if (m_input_motor2 < PULSE_WIDTH_US_MIN)
    {
        m_input_motor2 = (float)PULSE_WIDTH_US_MIN;
    }
    if (m_input_motor3 < PULSE_WIDTH_US_MIN)
    {
        m_input_motor3 = (float)PULSE_WIDTH_US_MIN;
    }
    if (m_input_motor4 < PULSE_WIDTH_US_MIN)
    {
        m_input_motor4 = (float)PULSE_WIDTH_US_MIN;
    }
}

void MotorsController::send_motor_inputs()
{
    write_pwm(MOTOR_1_ID, m_input_motor1);
    write_pwm(MOTOR_1_ID, m_input_motor1);
    write_pwm(MOTOR_1_ID, m_input_motor1);
    write_pwm(MOTOR_1_ID, m_input_motor1);
}

void MotorsController::write_us_to_pin(int pin, uint16_t us)
{
    uint32_t duty = map(us, 0, 20000, 0, (1 << PWM_RESOLUTION) - 1); // 20ms period
    ledcWrite(pin, duty);
}

uint16_t MotorsController::constrain_us(uint16_t us, uint16_t min, uint16_t max) const
{
    if (us < min)
    {
        return min;
    }
    if (us > max)
    {
        return max;
    }
    return us;
}
