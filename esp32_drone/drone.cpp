#include "drone.h"
#include "logger.h"

static const char *TAG = "DRONE";

Drone::Drone() : m_state(DroneState::INITIALIZING),
                 m_pid_roll(), m_pid_pitch(), m_pid_yaw() {}

Drone::~Drone() {}

void Drone::init()
{
    Wire.begin(DRONE_I2C_SDA, DRONE_I2C_SCL);

    /*** Initialize componenents ***/

    led_rgb_init();

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

    // if (!m_flight_receiver.init())
    // {
    //     change_state(DroneState::ERROR);
    //     return;
    // }

    // m_motors.init();
    // TODO: ESC sequence --> send 1000µs pwm for setup

    // Initialize PID controllers
    m_pid_roll.init(PID_GAIN_P_ROLL, PID_GAIN_I_ROLL, PID_GAIN_D_ROLL);
    m_pid_pitch.init(PID_GAIN_P_PITCH, PID_GAIN_I_PITCH, PID_GAIN_D_PITCH);
    m_pid_yaw.init(PID_GAIN_P_YAW, PID_GAIN_I_YAW, PID_GAIN_D_YAW);

    // Reset flags and variables
    m_emergency_protocol_engaged = false;
    m_last_time_sensors_checked = 0;

    // Atfer initialization, calibration required
    change_state(DroneState::CALIBRATING);
}

void Drone::update()
{
    uint32_t current_ms = millis();

    /*** Drone Finite State Machine ***/

    switch (m_state)
    {
    case DroneState::CALIBRATING:
        handle_state_calibrating();
        break;

    case DroneState::IDLE:
        handle_state_idle();
        break;

    case DroneState::DISARMED:
        handle_state_disarmed();
        break;

    case DroneState::ARMED:
        handle_state_armed();
        break;

    case DroneState::AUTO_TAKEOFF:
        handle_state_auto_takeoff();
        break;

    case DroneState::FLYING:
        handle_state_flying();
        break;

    case DroneState::AUTO_LANDING:
        handle_state_auto_landing();
        break;

    case DroneState::EMERGENCY_LANDING:
        handle_state_emergency_landing();
        break;

    case DroneState::ERROR:
        handle_state_error();
        break;

    default:
        break;
    }
}

void Drone::handle_state_calibrating()
{
    led_rgb_blue(ON);

    if (!m_imu.calibrate())
    {
        change_state(DroneState::ERROR);
    }
    else
    {
        change_state(DroneState::DISARMED);
    }
}

void Drone::handle_state_idle()
{
    led_rgb_white(ON);

    delay(DELAY_MS_IDLE);
    change_state(DroneState::DISARMED);
}

void Drone::handle_state_disarmed()
{
    // TODO: check sensors
    //       update FlightReceiver
    //       state --> ARMED when arm switch is up, for safety
    //       state --> ERROR if error occurs or emergency flag is up

    led_rgb_red(ON);

    if (m_emergency_protocol_engaged)
    {
        change_state(DroneState::ERROR);
        break;
    }

    if (!check_sensors())
    {
        change_state(DroneState::ERROR);
        break;
    }

    // update flightreceiver, check for switch ARMED
}

void Drone::handle_state_armed()
{
    // TODO: check sensors
    //       update FlightReceiver
    //       state --> AUTO_TAKEOFF when autotakeoff switch is up
    //       state --> FLIGHT when receiving throttle commands and other
    //       state --> ERROR if error occurs or emergency flag is up

    led_rgb_green(ON);

    if (!check_sensors())
    {
        change_state(DroneState::ERROR);
        break;
    }

    // Update flightreceiver, check for switch AUTO_TAKEOFF or check for throttle inputs with NEUTRAL
    // Spin lightly all 4 motors (5%~10%)
}

void Drone::handle_state_auto_takeoff()
{
    // TODO: update IMU readings
    //       Stabilize with PID
    //       Ascend slowly
    //       state --> FLIGHT when reached designated altitude, in cm
    //       state --> EMERGENCY_LANDING if error occurs

    led_rgb_blink(Color::PURPLE, LED_BLINK_INTERVAL_AUTO, current_ms);

    m_imu.update();
    m_tof.update();
    // stabilize_drone();
}

void Drone::handle_state_flying()
{
    // TODO: update IMU readings
    //       update FlightReceiver
    //       Stabilize with PID
    //       Respond to flight receiver commands with PWM for motors
    //       state --> AUTO_LANDING when autolanding switch is up
    //       state --> EMERGENCY_LANDING if error occurs

    m_imu.update();
}

void Drone::handle_state_auto_landing()
{
    // TODO: update IMU readings
    //       Stabilize with PID
    //       Descend slowly
    //       state --> DISARMED when reached ground (threshold in cm), for safety

    led_rgb_blink(Color::PURPLE, LED_BLINK_INTERVAL_AUTO, current_ms);

    m_imu.update();
    m_tof.update();
    // stabilize_drone();
}

void Drone::handle_state_emergency_landing()
{
    led_rgb_yellow(ON);

    // Start the emergency landing protocol: AUTO_LANDING -> DISARMED -> ERROR
    m_emergency_protocol_engaged = true;
    change_state(DroneState::AUTO_LANDING);
}

void Drone::handle_state_error()
{
    led_rgb_blink(Color::RED, LED_BLINK_INTERVAL_ERROR, current_ms);

    // TODO: Report sensors and other components status, emergency ?
}

bool Drone::check_sensors()
{
    // Delay check to not run every at every iteration
    if (millis() - m_last_time_sensors_checked >= DELAY_MS_CHECK_SENSORS)
    {
        m_last_time_sensors_checked = millis();

        if (!m_imu.check() || !m_tof.check())
        {
            logger.error(TAG, "Sensors failures detected");
            return false;
        }
    }

    return true;
}

void Drone::stabilize_drone()
{
    // Stabilize using PID controllers for roll, pitch, and yaw
    float roll_command = m_pid_roll.compute(m_imu.roll(), 0.0f);
    float pitch_command = m_pid_pitch.compute(m_imu.pitch(), 0.0f);
    float yaw_command = m_pid_yaw.compute(m_imu.yaw(), 0.0f);
}

const char *Drone::state_to_cstr(DroneState state)
{
    switch (state)
    {
    case DroneState::INITIALIZING:
        return "INITIALIZING";
    case DroneState::CALIBRATING:
        return "CALIBRATING";
    case DroneState::IDLE:
        return "IDLE";
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
