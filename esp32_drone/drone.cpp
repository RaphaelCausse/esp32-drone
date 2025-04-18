#include "drone.h"
#include "logger.h"

static const char *TAG = "DRONE";

Drone::Drone() : m_state(DroneState::INITIALIZING),
                 m_imu(), m_tof(),
                 m_motors(),
                 m_pid_roll(), m_pid_pitch(), m_pid_yaw() {}

Drone::~Drone() {}

void Drone::init()
{
    Wire.begin(DRONE_I2C_SDA, DRONE_I2C_SCL);

    /*** Initialize componenents ***/

    led_rgb_init();
    led_rgb_white(ON);

    if (!m_imu.init(&Wire))
    {
        change_state(DroneState::ERROR);
        return;
    }

    if (!m_tof.init(&Wire))
    {
        change_state(DroneState::ERROR);
        return;
    }

    if (!m_flight_receiver.init())
    {
        change_state(DroneState::ERROR);
        return;
    }

    if (!m_motors.init())
    {
        change_state(DroneState::ERROR);
        return;
    }

    // Initialize PID controllers
    m_pid_roll.init(PID_GAIN_P_ROLL, PID_GAIN_I_ROLL, PID_GAIN_D_ROLL);
    m_pid_pitch.init(PID_GAIN_P_PITCH, PID_GAIN_I_PITCH, PID_GAIN_D_PITCH);
    m_pid_yaw.init(PID_GAIN_P_YAW, PID_GAIN_I_YAW, PID_GAIN_D_YAW);

    // Reset flags and variables
    m_emergency_protocol_engaged = false;
    m_last_sensors_checked_ms = 0;
    m_last_pwm_update_ms = 0;

    // Atfer initialization, calibration required
    change_state(DroneState::CALIBRATING);
}

void Drone::update()
{
    uint32_t current_ms = millis();

    // Update commands from receiver before handling state
    m_flight_receiver.update();

    /*** Drone Finite State Machine ***/

    switch (m_state)
    {
    case DroneState::CALIBRATING:
        handle_state_calibrating(current_ms);
        break;

    case DroneState::DISARMED:
        handle_state_disarmed(current_ms);
        break;

    case DroneState::ARMED:
        handle_state_armed(current_ms);
        break;

    case DroneState::AUTO_TAKEOFF:
        handle_state_auto_takeoff(current_ms);
        break;

    case DroneState::FLYING:
        handle_state_flying(current_ms);
        break;

    case DroneState::AUTO_LANDING:
        handle_state_auto_landing(current_ms);
        break;

    case DroneState::EMERGENCY_LANDING:
        handle_state_emergency_landing(current_ms);
        break;

    case DroneState::ERROR:
        handle_state_error(current_ms);
        break;

    default:
        break;
    }

    // Loop duration control
    while (millis() - current_ms < LOOP_DURATION_MS)
    {
    }
}

void Drone::handle_state_calibrating(uint32_t current_ms)
{
    led_rgb_blue(ON);

    // IMU calibration
    if (!m_imu.calibrate())
    {
        change_state(DroneState::ERROR);
    }

    // change_state(DroneState::ARMED);
    change_state(DroneState::DISARMED);
}

void Drone::handle_state_disarmed(uint32_t current_ms)
{
    led_rgb_red(ON);

    // Safety enabled, stop motors
    if (current_ms - m_last_pwm_update_ms >= DELAY_MS_SEND_PWM)
    {
        m_last_pwm_update_ms = current_ms;
        m_motors.stop_motors(DURATION_S_STOP_MOTOR_FAST);
    }

    if (m_emergency_protocol_engaged || !check_sensors())
    {
        change_state(DroneState::ERROR);
        return;
    }

    if (m_flight_receiver.is_armed())
    {
        change_state(DroneState::ARMED);
    }
}

void Drone::handle_state_armed(uint32_t current_ms)
{
    led_rgb_green(ON);

    if (m_emergency_protocol_engaged || m_flight_receiver.is_disarmed())
    {
        m_pid_roll.reset();
        m_pid_pitch.reset();
        m_pid_yaw.reset();
        change_state(DroneState::DISARMED);
        return;
    }

    if (!check_sensors())
    {
        change_state(DroneState::ERROR);
        return;
    }

    if (m_flight_receiver.is_armed())
    {
        if (m_flight_receiver.is_auto_takeoff())
        {
            change_state(DroneState::AUTO_TAKEOFF);
            return;
        }

        if (m_flight_receiver.is_neutral())
        {
            // Manual takeoff engaged
            if (m_flight_receiver.target_throttle() > (float)MotorsController::PULSE_WIDTH_US_SPIN_ARMED)
            {
                change_state(DroneState::FLYING);
                return;
            }
        }

        // Safety disabled, spin motor
        current_ms = millis();
        if (current_ms - m_last_pwm_update_ms >= DELAY_MS_SEND_PWM)
        {
            m_last_pwm_update_ms = current_ms;
            m_motors.spin_motors_armed();
            m_current_throttle = (float)MotorsController::PULSE_WIDTH_US_SPIN_ARMED;
        }
    }
}

void Drone::handle_state_auto_takeoff(uint32_t current_ms)
{
    led_rgb_blink(Color::PURPLE, LED_BLINK_INTERVAL_NORMAL, current_ms);

    if (!check_sensors() || m_flight_receiver.is_disarmed())
    {
        change_state(DroneState::EMERGENCY_LANDING);
        return;
    }

    if (m_flight_receiver.is_armed())
    {
        if (m_flight_receiver.is_auto_landing())
        {
            change_state(DroneState::AUTO_LANDING);
            return;
        }

        if (m_flight_receiver.is_neutral())
        {
            // Manual flying
            if (m_flight_receiver.target_throttle() > (float)MotorsController::PULSE_WIDTH_US_SPIN_HALF)
            {
                change_state(DroneState::FLYING);
                return;
            }
        }

        // Auto Takeoff logic
        if (m_flight_receiver.is_auto_takeoff())
        {
            m_imu.update();
            float roll_measured = m_imu.roll();
            float pitch_measured = m_imu.pitch();
            float yaw_measured = m_imu.yaw();

            m_tof.update();
            m_current_altitude = m_tof.distance_cm();
            logger.debug(TAG, "Altitude: %.2f cm", m_current_altitude);

            if ((m_current_altitude < TARGET_ALTITUDE_CM) && (m_current_altitude != -1.0f))
            {
                // Climb
                m_current_throttle += RATE_PWM_US_CLIMB;
                if (m_current_throttle > (float)MotorsController::PULSE_WIDTH_US_SPIN_MAX)
                {
                    m_current_throttle = (float)MotorsController::PULSE_WIDTH_US_SPIN_MAX;
                }
            }
            else if ((m_current_altitude >= TARGET_ALTITUDE_CM) && (m_current_altitude != -1.0f))
            {
                // Reached target altitude, give back control to pilot for manual flight
                change_state(DroneState::FLYING);
                return;
            }

            // Compute PID, targets 0.0f to stabilize
            float roll_correction = m_pid_roll.compute(roll_measured, 0.0f);
            float pitch_correction = m_pid_pitch.compute(pitch_measured, 0.0f);
            float yaw_correction = m_pid_yaw.compute(yaw_measured, 0.0f);

            // Always keep minimum idle spin speed
            if (m_current_throttle < (float)MotorsController::PULSE_WIDTH_US_SPIN_IDLE)
            {
                m_current_throttle = (float)MotorsController::PULSE_WIDTH_US_SPIN_IDLE;
            }

            // Compute and send motors inputs
            m_motors.compute_motor_inputs(m_current_throttle, roll_correction, pitch_correction, yaw_correction);
            current_ms = millis();
            if (current_ms - m_last_pwm_update_ms >= DELAY_MS_SEND_PWM)
            {
                m_last_pwm_update_ms = current_ms;
                m_motors.send_motor_inputs();
            }
        }
    }
}

void Drone::handle_state_flying(uint32_t current_ms)
{
    led_rgb_green(ON);

    if (!check_sensors())
    {
        change_state(DroneState::EMERGENCY_LANDING);
        return;
    }

    m_tof.update();
    m_current_altitude = m_tof.distance_cm();
    logger.debug(TAG, "Altitude: %.2f cm", m_current_altitude);

    // If altitude below threshold, drone reached ground
    if (m_flight_receiver.is_disarmed() &&
        (m_current_altitude < LANDING_THRESHOLD_CM) &&
        (m_current_altitude != -1.0f))
    {
        m_motors.stop_motors(DURATION_S_STOP_MOTOR_NORMAL);
        m_pid_roll.reset();
        m_pid_pitch.reset();
        m_pid_yaw.reset();
        change_state(DroneState::DISARMED);
        return;
    }

    if (m_flight_receiver.is_armed())
    {
        if (m_flight_receiver.is_auto_landing())
        {
            change_state(DroneState::AUTO_LANDING);
            return;
        }

        // Manual flying logic
        if (m_flight_receiver.is_neutral())
        {
            m_imu.update();
            float roll_measured = m_imu.roll();
            float pitch_measured = m_imu.pitch();
            float yaw_measured = m_imu.yaw();

            // Get pilot commands
            float throttle_input = m_flight_receiver.target_throttle();
            float roll_target_rate = m_flight_receiver.target_roll_rate();   // in °/s
            float pitch_target_rate = m_flight_receiver.target_pitch_rate(); // in °/s
            float yaw_target_rate = m_flight_receiver.target_yaw_rate();     // in °/s

            // Compute PID corrections to follow target
            float roll_correction = m_pid_roll.compute(roll_measured, roll_target_rate);
            float pitch_correction = m_pid_pitch.compute(pitch_measured, pitch_target_rate);
            float yaw_correction = m_pid_yaw.compute(yaw_measured, yaw_target_rate);

            // Always keep minimum idle spin speed
            if (throttle_input < (float)MotorsController::PULSE_WIDTH_US_SPIN_IDLE)
            {
                throttle_input = (float)MotorsController::PULSE_WIDTH_US_SPIN_IDLE;
            }

            // Compute and send motors inputs
            m_motors.compute_motor_inputs(throttle_input, roll_correction, pitch_correction, yaw_correction);
            m_motors.send_motor_inputs();
            current_ms = millis();
            if (current_ms - m_last_pwm_update_ms >= DELAY_MS_SEND_PWM)
            {
                m_last_pwm_update_ms = current_ms;
                m_motors.send_motor_inputs();
            }
        }
    }
}

void Drone::handle_state_auto_landing(uint32_t current_ms)
{
    if (m_emergency_protocol_engaged)
    {
        led_rgb_yellow(ON);

        // Emergency protocole, land without sensors, disarm
        m_motors.stop_motors(DURATION_S_STOP_MOTOR_NORMAL);
        m_pid_roll.reset();
        m_pid_pitch.reset();
        m_pid_yaw.reset();
        change_state(DroneState::DISARMED);
        return;
    }

    led_rgb_blink(Color::PURPLE, LED_BLINK_INTERVAL_NORMAL, current_ms);

    if (!check_sensors() || m_flight_receiver.is_disarmed())
    {
        change_state(DroneState::EMERGENCY_LANDING);
        return;
    }

    if (m_flight_receiver.is_armed())
    {
        if (m_flight_receiver.is_neutral())
        {
            change_state(DroneState::FLYING);
            return;
        }

        // Auto landing logic
        if (m_flight_receiver.is_auto_landing())
        {
            m_imu.update();
            float roll_measured = m_imu.roll();
            float pitch_measured = m_imu.pitch();
            float yaw_measured = m_imu.yaw();

            m_tof.update();
            m_current_altitude = m_tof.distance_cm();
            logger.debug(TAG, "Altitude: %.2f cm", m_current_altitude);

            if ((m_current_altitude > LANDING_THRESHOLD_CM) && (m_current_altitude != -1.0f))
            {
                // Descend
                m_current_throttle -= RATE_PWM_US_DESCEND;
                if (m_current_throttle < (float)MotorsController::PULSE_WIDTH_US_SPIN_IDLE)
                {
                    m_current_throttle = (float)MotorsController::PULSE_WIDTH_US_SPIN_IDLE;
                }
            }
            else if ((m_current_altitude <= LANDING_THRESHOLD_CM) && (m_current_altitude != -1.0f))
            {
                // Readched ground, can stop motors and disarm
                m_motors.stop_motors(DURATION_S_STOP_MOTOR_NORMAL);
                m_pid_roll.reset();
                m_pid_pitch.reset();
                m_pid_yaw.reset();
                change_state(DroneState::DISARMED);
                return;
            }

            // Compute PID, targets 0.0f to stabilize
            float roll_correction = m_pid_roll.compute(roll_measured, 0.0f);
            float pitch_correction = m_pid_pitch.compute(pitch_measured, 0.0f);
            float yaw_correction = m_pid_yaw.compute(yaw_measured, 0.0f);

            // Compute and send motors inputs
            m_motors.compute_motor_inputs(m_current_throttle, roll_correction, pitch_correction, yaw_correction);
            current_ms = millis();
            if (current_ms - m_last_pwm_update_ms >= DELAY_MS_SEND_PWM)
            {
                m_last_pwm_update_ms = current_ms;
                m_motors.send_motor_inputs();
            }
        }
    }
}

void Drone::handle_state_emergency_landing(uint32_t current_ms)
{
    led_rgb_yellow(ON);

    // Engage emergency landing protocol: AUTO_LANDING -> DISARMED -> ERROR
    m_emergency_protocol_engaged = true;
    change_state(DroneState::AUTO_LANDING);
}

void Drone::handle_state_error(uint32_t current_ms)
{
    led_rgb_blink(Color::RED, LED_BLINK_INTERVAL_FAST, current_ms);

    // TODO: Report sensors and other components status, emergency ?
}

bool Drone::check_sensors()
{
    // Delay check to not run every at every iteration
    if (millis() - m_last_sensors_checked_ms >= DELAY_MS_CHECK_SENSORS)
    {
        m_last_sensors_checked_ms = millis();

        if (!m_imu.check() || !m_tof.check())
        {
            logger.error(TAG, "Sensors failures detected");
            return false;
        }
    }

    return true;
}

const char *Drone::state_to_cstr(DroneState state)
{
    switch (state)
    {
    case DroneState::INITIALIZING:
        return "INITIALIZING";
    case DroneState::CALIBRATING:
        return "CALIBRATING";
    case DroneState::DISARMED:
        return "DISARMED";
    case DroneState::ARMED:
        return "ARMED";
    case DroneState::AUTO_TAKEOFF:
        return "AUTO_TAKEOFF";
    case DroneState::FLYING:
        return "FLYING";
    case DroneState::AUTO_LANDING:
        return "AUTO_LANDING";
    case DroneState::EMERGENCY_LANDING:
        return "EMERGENCY_LANDING";
    case DroneState::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

void Drone::change_state(DroneState new_state)
{
    led_rgb_unset();
    logger.info(TAG, "Change state: %s -> %s", state_to_cstr(m_state), state_to_cstr(new_state));
    m_state = new_state;
}
