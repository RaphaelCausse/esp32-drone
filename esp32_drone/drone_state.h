#ifndef ESP32_DRONE_DRONE_STATE_H
#define ESP32_DRONE_DRONE_STATE_H

enum class DroneState
{
    INITIALIZING, // Initializing sensors, receiver, motors, etc.
    CALIBRATING,  // Calibrating sensors

    DISARMED, // Safety enabled, motors are off, waiting to be armed
    ARMED,    // Safety disabled, motors are ready

    AUTO_TAKEOFF, // Autonomous takeoff (PID)
    FLYING,       // In-flight (PID)
    AUTO_LANDING, // Autonomous landing (PID)

    EMERGENCY_LANDING, // Emergency: lost signal or sensors, or critical failure
    ERROR              // Critical failure: flight is not allowed
};

#endif /* ESP32_DRONE_DRONE_STATE_H */
