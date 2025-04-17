#include <WiFi.h>

#include "flight_receiver.h"
#include "logger.h"

static const char *TAG = "FLIGHT_RCVR";

struct_command cmd;

void espnow_receive_callback(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // READ values in m_received_values convert to float
    memcpy(&cmd, incomingData, sizeof(cmd));
    logger.info(TAG, "roll=%d \t| pitch=%d \t| throttle=%d \t| yaw=%d \t| arm=%d \t| mode=%d", cmd.channel1, cmd.channel2, cmd.channel3, cmd.channel4, cmd.switch_disarmed_armed, cmd.switch_mode);
}

FlightReceiver::FlightReceiver() {}

FlightReceiver::~FlightReceiver() {}

bool FlightReceiver::init()
{
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      logger.error(TAG, "Error initializing ESP-NOW");
      return false;
    }

    // Once ESPNow is successfully Init, we will register for recv CB
    // to get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(espnow_receive_callback));
    return true;
}

float FlightReceiver::throttle() const
{
    return (float)cmd.channel3;
}

float FlightReceiver::target_roll_rate() const
{
    return (float)(COEF_ROTATION_RATE * (float)(cmd.channel1 - LINEAR_CORRELATION));
}

float FlightReceiver::target_pitch_rate() const
{
    return (float)(COEF_ROTATION_RATE * (float)(cmd.channel2 - LINEAR_CORRELATION));
}

float FlightReceiver::target_yaw_rate() const
{
    return (float)(COEF_ROTATION_RATE * (float)(cmd.channel4 - LINEAR_CORRELATION));
}

bool FlightReceiver::is_armed() const
{
    return (cmd.switch_disarmed_armed == RECEIVE_VALUE_MAX);
}

bool FlightReceiver::is_disarmed() const
{
    return (cmd.switch_disarmed_armed == RECEIVE_VALUE_MIN);
}

bool FlightReceiver::is_neutral() const
{
    return (cmd.switch_mode == LINEAR_CORRELATION);
}

bool FlightReceiver::is_auto_landing() const
{
    return (cmd.switch_mode == RECEIVE_VALUE_MIN);
}

bool FlightReceiver::is_auto_take_off() const
{
    return (cmd.switch_mode == RECEIVE_VALUE_MAX);
}