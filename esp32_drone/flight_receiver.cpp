#include <algorithm>
#include <WiFi.h>
#include "flight_receiver.h"
#include "logger.h"

static const char *TAG = "RECEIVER";

struct_command cmd;

void espnow_receive_callback(const uint8_t *mac, const uint8_t *incoming_data, int len)
{
    // READ values in m_received_values convert to float
    memcpy(&cmd, incoming_data, sizeof(cmd));
    // logger.debug(TAG, "roll=%d \t| pitch=%d \t| throttle=%d \t| yaw=%d \t| arm=%d \t| mode=%d", cmd.roll, cmd.pitch, cmd.throttle, cmd.yaw, cmd.switch_arm, cmd.switch_mode);
}

FlightReceiver::FlightReceiver() {}

FlightReceiver::~FlightReceiver() {}

bool FlightReceiver::init()
{
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        logger.error(TAG, "Error initializing ESP-NOW");
        return false;
    }

    // Register for receive callback to get received info
    esp_now_register_recv_cb(esp_now_recv_cb_t(espnow_receive_callback));

    logger.info(TAG, "Flight Receiver (ESPNOW) initialized");
    return true;
}

void FlightReceiver::update()
{
    // Check if Arm switch is manually set to Disarmed
    if (cmd.switch_arm == RECEIVE_VALUE_MIN)
    {
        m_was_first_disarmed = true;
    }
}

float FlightReceiver::target_throttle() const
{
    return clamp(cmd.throttle, RECEIVE_VALUE_MIN, MAX_THROTTLE);
}

float FlightReceiver::target_roll_rate() const
{

    return (float)(COEF_ROTATION_RATE * (float)(cmd.roll - RECEIVE_VALUE_MID));
}

float FlightReceiver::target_pitch_rate() const
{
    return (float)(COEF_ROTATION_RATE * (float)(cmd.pitch - RECEIVE_VALUE_MID));
}

float FlightReceiver::target_yaw_rate() const
{
    return (float)(COEF_ROTATION_RATE * (float)(cmd.yaw - RECEIVE_VALUE_MID));
}

bool FlightReceiver::is_armed() const
{
    return (m_was_first_disarmed &&
            (cmd.switch_arm == RECEIVE_VALUE_MAX) &&
            (cmd.switch_arm >= RECEIVE_VALUE_MIN));
}

bool FlightReceiver::is_disarmed() const
{
    return ((cmd.switch_arm == RECEIVE_VALUE_MIN) && (cmd.switch_arm >= RECEIVE_VALUE_MIN));
}

bool FlightReceiver::is_neutral() const
{
    return ((cmd.switch_mode == RECEIVE_VALUE_MID) && (cmd.switch_mode >= RECEIVE_VALUE_MIN));
}

bool FlightReceiver::is_auto_landing() const
{
    return ((cmd.switch_mode == RECEIVE_VALUE_MIN) && (cmd.switch_mode >= RECEIVE_VALUE_MIN));
}

bool FlightReceiver::is_auto_takeoff() const
{
    return ((cmd.switch_mode == RECEIVE_VALUE_MAX) && (cmd.switch_mode >= RECEIVE_VALUE_MIN));
}

float FlightReceiver::clamp(uint16_t value, uint16_t min, uint16_t max) const
{
    if (value > max)
    {
        return (float)max;
    }
    if (value < min)
    {
        return (float)min;
    }
    return (float)value;
}