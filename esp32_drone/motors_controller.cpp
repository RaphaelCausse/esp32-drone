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

    // Initialize ESC with PWM 1000µs signal
    logger.info(TAG, "Send init PWM signal to all motors (1000µs)");
    stop_all();
    delay(DELAY_INIT_ESC);
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

    switch (motor_id)
    {
    case MOTOR_1_ID:
        write_us_to_pin(MOTOR_1_PIN, us);
        break;
    case MOTOR_2_ID:
        write_us_to_pin(MOTOR_2_PIN, us);
        break;
    case MOTOR_3_ID:
        write_us_to_pin(MOTOR_3_PIN, us);
        break;
    case MOTOR_4_ID:
        write_us_to_pin(MOTOR_4_PIN, us);
        break;
    default:
        logger.warning(TAG, "Invalid motor ID: %d", motor_id);
        break;
    }
}

void MotorsController::stop_all()
{
    write_pwm(MOTOR_1_ID, MIN_PULSE_WIDTH_US);
    write_pwm(MOTOR_2_ID, MIN_PULSE_WIDTH_US);
    write_pwm(MOTOR_3_ID, MIN_PULSE_WIDTH_US);
    write_pwm(MOTOR_4_ID, MIN_PULSE_WIDTH_US);
}

void MotorsController::spin_idle(uint32_t current_ms)
{
    uint16_t spin_pulse = MIN_PULSE_WIDTH_US + (MIN_PULSE_WIDTH_US * SPIN_ARMED_PERCENTAGE / 100);
    // logger.debug(TAG, "Arming motors to %d%% (%u µs)...", SPIN_IDLE_PERCENTAGE, spin_pulse);
    write_pwm(MOTOR_1_ID, spin_pulse);
    write_pwm(MOTOR_2_ID, spin_pulse);
    write_pwm(MOTOR_3_ID, spin_pulse);
    write_pwm(MOTOR_4_ID, spin_pulse);
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
