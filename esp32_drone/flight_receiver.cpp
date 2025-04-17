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
    return (float)(COEF_ROTATION_RATE * (float)(m_received_values[IDX_ROLL] - LINEAR_CORRELATION));
}

float FlightReceiver::target_pitch_rate() const
{
    return (float)(COEF_ROTATION_RATE * (float)(m_received_values[IDX_PITCH] - LINEAR_CORRELATION));
}

float FlightReceiver::target_yaw_rate() const
{
    return (float)(COEF_ROTATION_RATE * (float)(m_received_values[IDX_YAW] - LINEAR_CORRELATION));
}

bool FlightReceiver::is_armed() const
{
    return (m_received_values[IDX_ARM] == RECEIVE_VALUE_MAX);
}

bool FlightReceiver::is_disarmed() const
{
    return (m_received_values[IDX_ARM] == RECEIVE_VALUE_MIN);
}
