#ifndef ESP32_DRONE_FLIGHT_RECEIVER_H
#define ESP32_DRONE_FLIGHT_RECEIVER_H

#include <Arduino.h>

class FlightReceiver
{
public:
    FlightReceiver();
    ~FlightReceiver();

    void init();
    void update();

    float throttle() const;
    float target_roll_rate() const;
    float target_pitch_rate() const;
    float target_yaw_rate() const;
    bool is_armed() const;
    bool is_disarmed() const;
    // TODO is_mode ...

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
