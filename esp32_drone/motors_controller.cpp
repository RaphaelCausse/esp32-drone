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
    us = constrain_us(us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

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
    logger.info(TAG, "Send init PWM signal to all motors (1000µs)");
    write_pwm(MOTOR_1_ID, MIN_PULSE_WIDTH_US);
    write_pwm(MOTOR_2_ID, MIN_PULSE_WIDTH_US);
    write_pwm(MOTOR_3_ID, MIN_PULSE_WIDTH_US);
    write_pwm(MOTOR_4_ID, MIN_PULSE_WIDTH_US);
    delay(DELAY_INIT_ESC);
}

void MotorsController::stop_all()
{
    const int steps = 3;
    const int delay_ms = 50;

    // Progressive stop
    for (int s = 0; s < steps; ++s)
    {
        for (int i = 0; i < 4; ++i)
        {
            uint16_t current_pwm = m_motor_last_pwm[i];
            uint16_t delta = (current_pwm - MIN_PULSE_WIDTH_US) / (steps - s);
            uint16_t new_pwm = current_pwm - delta;
            if (new_pwm < MIN_PULSE_WIDTH_US)
            {
                new_pwm = MIN_PULSE_WIDTH_US;
            }

            write_pwm(i + 1, new_pwm);
            m_motor_last_pwm[i] = new_pwm;
        }
        delay(delay_ms);
    }

    // Make sure to shutdown motors
    for (int i = 0; i < 4; ++i)
    {
        write_pwm(i + 1, MIN_PULSE_WIDTH_US);
        m_motor_last_pwm[i] = MIN_PULSE_WIDTH_US;
    }
}

void MotorsController::spin_armed()
{
    uint16_t spin_pulse = MIN_PULSE_WIDTH_US + (MIN_PULSE_WIDTH_US * SPIN_ARMED_PERCENTAGE / 100);
    write_pwm(MOTOR_1_ID, spin_pulse);
    write_pwm(MOTOR_2_ID, spin_pulse);
    write_pwm(MOTOR_3_ID, spin_pulse);
    write_pwm(MOTOR_4_ID, spin_pulse);
}

void MotorsController::compute_motor_inputs(float input_throttle, float input_roll, float input_pitch, float input_yaw)
{
    m_input_motor1 = input_throttle - input_roll - input_pitch - input_yaw;
    m_input_motor2 = input_throttle - input_roll + input_pitch + input_yaw;
    m_input_motor2 = input_throttle + input_roll + input_pitch - input_yaw;
    m_input_motor2 = input_throttle + input_roll - input_pitch + input_yaw;
}

void MotorsController::write_us_to_pin(int pin, uint16_t us)
{
    uint32_t duty = map(us, 0, 20000, 0, (1 << PWM_RESOLUTION) - 1); // 20ms period
    ledcWrite(pin, duty);
}

uint16_t MotorsController::constrain_us(uint16_t us, uint16_t min, uint16_t max)
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
