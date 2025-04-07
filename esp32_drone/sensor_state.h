#ifndef ESP32_DRONE_SENSOR_STATE_H
#define ESP32_DRONE_SENSOR_STATE_H

enum class SensorState
{
    INACTIVE, // The sensor is inactive
    ACTIVE,   // The sensor is active and functional
    ERROR     // The sensor is encountering an issue
};

#endif /* ESP32_DRONE_SENSOR_STATE_H */
