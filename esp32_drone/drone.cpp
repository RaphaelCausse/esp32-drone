#include <string>
#include "drone.h"
#include "logger.h"

static const char *TAG = "DRONE";

Drone::Drone() : m_state(DroneState::INITIALIZING) {}

Drone::~Drone() {}

void Drone::init()
{
    led_rgb_init();
    Wire.begin(DRONE_I2C_SDA, DRONE_I2C_SCL);

    /*** Initialize critical componenents ***/

    if (!m_imu.init())
    {
        m_state = DroneState::ERROR;
        return;
    }
    // TODO: initialize FlightReceiver

    /*** Initialize non-critical components ***/

    if (!m_tof.init())
    {
        m_state = DroneState::DEGRADED;
        return;
    }

    change_state(DroneState::CALIBRATING);
}

void Drone::update()
{
    uint32_t current_ms = millis();

    switch (m_state)
    {
    case DroneState::CALIBRATING:
        led_rgb_blue(ON);
        m_imu.calibrate();
        change_state(DroneState::IDLE);
        break;

    case DroneState::IDLE:
        led_rgb_blink(Color::WHITE, LED_BLINK_INTERVAL_IDLE, current_ms);
        // TODO
        break;

    case DroneState::DISARMED:
        led_rgb_blink(Color::RED, LED_BLINK_INTERVAL_DISARMED, current_ms);
        // TODO
        break;

    case DroneState::ARMED:
        led_rgb_blink(Color::GREEN, LED_BLINK_INTERVAL_ARMED, current_ms);
        // TODO
        break;

    case DroneState::AUTO_TAKEOFF:
        // TODO
        break;

    case DroneState::FLYING:
        // TODO
        break;

    case DroneState::AUTO_LANDING:
        // TODO
        break;

    case DroneState::EMERGENCY_LANDING:
        // TODO
        break;

    case DroneState::DEGRADED:
        led_rgb_yellow(ON);
        // TODO
        break;

    case DroneState::ERROR:
        led_rgb_blink(Color::RED, LED_BLINK_INTERVAL_ERROR, current_ms);
        // TODO
        break;

    default:
        break;
    }
}

const char *Drone::state_to_str(DroneState state)
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
    case DroneState::DEGRADED:
        return "DEGRADED";
    case DroneState::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

void Drone::change_state(DroneState new_state)
{
    led_rgb_unset();
    logger.info(TAG, "Change state: %s -> %s", state_to_str(m_state), state_to_str(new_state));
    m_state = new_state;
}