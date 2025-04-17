#include "flight_receiver.h"

static const char *TAG = "FLIGHT_RCVR";

FlightReceiver::FlightReceiver() {}

FlightReceiver::~FlightReceiver() {}

void FlightReceiver::init()
{
    // ESPNOW
}

void FlightReceiver::update()
{
    // READ values in m_received_values convert to float
}

float FlightReceiver::throttle() const
{
    return (float)m_received_values[IDX_THROTTLE];
}

float FlightReceiver::target_roll_rate() const
{
    return compute_target_rotation_rate(m_received_values[IDX_ROLL]);
}

float FlightReceiver::target_pitch_rate() const
{
    return compute_target_rotation_rate(m_received_values[IDX_PITCH]);
}

float FlightReceiver::target_yaw_rate() const
{
    return compute_target_rotation_rate(m_received_values[IDX_YAW]);
}

bool FlightReceiver::is_armed() const
{
    return (m_received_values[IDX_ARM] == 2000);
}

float FlightReceiver::compute_target_rotation_rate(uint16_t value)
{
    return (float)(COEF_ROTATION_RATE * (float)(value - LINEAR_CORRELATION));
}