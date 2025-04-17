#ifndef ESP32_DRONE_FLIGHT_RECEIVER_H
#define ESP32_DRONE_FLIGHT_RECEIVER_H

#include <Arduino.h>
#include <esp_now.h>

struct struct_command {
  uint16_t channel1; //roll
  uint16_t channel2; //pitch
  uint16_t channel3; //throttle
  uint16_t channel4; //yaw
  uint16_t switch_disarmed_armed;
  uint16_t switch_mode;
};

class FlightReceiver
{
public:
    FlightReceiver();
    ~FlightReceiver();

    bool init();

    float throttle() const;
    float target_roll_rate() const;
    float target_pitch_rate() const;
    float target_yaw_rate() const;
    bool is_armed() const;
    bool is_disarmed() const;
    bool is_neutral() const;
    bool is_auto_landing() const;
    bool is_auto_take_off() const;

private:
    uint16_t m_received_values[6];

private:
    static constexpr int IDX_ROLL = 0;
    static constexpr int IDX_PITCH = 1;
    static constexpr int IDX_THROTTLE = 2;
    static constexpr int IDX_YAW = 3;
    static constexpr int IDX_ARM = 4;
    static constexpr int IDX_MODE = 5;

    static constexpr uint16_t RECEIVE_VALUE_MAX = 2000;
    static constexpr uint16_t RECEIVE_VALUE_MIN = 1000;

    static constexpr float COEF_ROTATION_RATE = 0.15;
    static constexpr uint16_t LINEAR_CORRELATION = 1500;
};

#endif /* ESP32_DRONE_FLIGHT_RECEIVER_H */
