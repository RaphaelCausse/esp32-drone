#include "drone.h"
#include "logger.h"

static const char *TAG = "DRONE";

Drone::Drone() : m_state(DroneState::INITIALIZING),
                 m_pid_roll(), m_pid_pitch(), m_pid_yaw() {}

Drone::~Drone() {}

void Drone::init()
{
    led_rgb_init();
    Wire.begin(DRONE_I2C_SDA, DRONE_I2C_SCL);

    /*** Initialize componenents ***/

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

    // Initialize PID controllers
    m_pid_roll.init(PID_GAIN_P_ROLL, PID_GAIN_I_ROLL, PID_GAIN_D_ROLL);
    m_pid_pitch.init(PID_GAIN_P_PITCH, PID_GAIN_I_PITCH, PID_GAIN_D_PITCH);
    m_pid_yaw.init(PID_GAIN_P_YAW, PID_GAIN_I_YAW, PID_GAIN_D_YAW);

    // Reset flags
    m_emergency_protocol_engaged = false;

    // Calibration required
    change_state(DroneState::CALIBRATING);
}

void Drone::update()
{
    uint32_t current_ms = millis();

    /*** State machine ***/

    switch (m_state)
    {
    case DroneState::CALIBRATING:
    {
        led_rgb_blue(ON);
        if (!m_imu.calibrate())
        {
            logger.error(TAG, "Failed to calibrate IMU");
            change_state(DroneState::ERROR);
        }
        else
        {
            change_state(DroneState::DISARMED);
        }
    }
    break;

    case DroneState::IDLE:
    {
        led_rgb_white(ON);
        delay(DELAY_IDLE);
        change_state(DroneState::DISARMED);
    }
    break;

    case DroneState::DISARMED:
    {
        // TODO: update IMU readings
        //       state --> ARMED when arm switch is up, for safety
        //       state --> ERROR if error occurs or emergency flag is up

        led_rgb_red(ON);

        m_imu.update();

        if (m_emergency_protocol_engaged)
        {
            change_state(DroneState::ERROR);
        }
    }
    break;

    case DroneState::ARMED:
    {
        // TODO: update IMU readings
        //       update FlightReceiver
        //       state --> AUTO_TAKEOFF when autotakeoff switch is up
        //       state --> FLIGHT when receiving throttle commands and other
        //       state --> ERROR if error occurs or emergency flag is up

        led_rgb_green(ON);

        m_imu.update();
    }
    break;

    case DroneState::AUTO_TAKEOFF:
    {
        // TODO: update IMU readings
        //       Stabilize with PID
        //       state --> FLIGHT when reached designated altitude, in cm
        //       state --> EMERGENCY_LANDING if error occurs

        led_rgb_blink(Color::PURPLE, LED_BLINK_INTERVAL_AUTO, current_ms);

        m_imu.update();
        stabilize_drone();
    }
    break;

    case DroneState::FLYING:
    {
        // TODO: update IMU readings
        //       update FlightReceiver
        //       Stabilize with PID
        //       Respond to flight receiver commands with PWM for motors
        //       state --> AUTO_LANDING when autolanding switch is up
        //       state --> EMERGENCY_LANDING if error occurs

        m_imu.update();
    }
    break;

    case DroneState::AUTO_LANDING:
    {
        // TODO: update IMU readings
        //       Stabilize with PID
        //       state --> DISARMED when reached ground (threshold in cm), for safety

        led_rgb_blink(Color::PURPLE, LED_BLINK_INTERVAL_AUTO, current_ms);

        m_imu.update();
        stabilize_drone();
    }
    break;

    case DroneState::EMERGENCY_LANDING:
    {
        // Start the emergency landing protocol: AUTO_LANDING -> DISARMED -> ERROR
        m_emergency_protocol_engaged = true;
        change_state(DroneState::AUTO_LANDING);
    }
    break;

    case DroneState::ERROR:
    {
        // TODO: Report sensors and other components status, emergency ?

        led_rgb_blink(Color::RED, LED_BLINK_INTERVAL_ERROR, current_ms);
    }
    break;

    default:
        break;
    }
}

void Drone::stabilize_drone()
{
    // Stabilize using PID controllers for roll, pitch, and yaw
    float roll_command = m_pid_roll.compute(m_imu.roll(), 0.0f);
    float pitch_command = m_pid_pitch.compute(m_imu.pitch(), 0.0f);
    float yaw_command = m_pid_yaw.compute(m_imu.yaw(), 0.0f);

    // TODO: maybe change PIDController class so commands are within the class.
    //       => to separate pid computation and motor signals ?
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
