#ifndef ESP32_DRONE_SENSOR_STATE_H
#define ESP32_DRONE_SENSOR_STATE_H

enum class SensorState
{
    INACTIVE, // Sensor is inactive
    ACTIVE,   // Sensor is active and functional
    ERROR     // Sensor is encountering an issue
};

#endif /* ESP32_DRONE_SENSOR_STATE_H */
