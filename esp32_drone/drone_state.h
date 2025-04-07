#ifndef ESP32_DRONE_DRONE_STATE_H
#define ESP32_DRONE_DRONE_STATE_H

enum class DroneState
{
    INITIALIZING, // Initializing sensors, receiver, motors, etc.
    CALIBRATING,  // Calibrating sensor

    IDLE,     // Drone is ready but waiting for user action
    DISARMED, // Safety enabled, motors are off
    ARMED,    // Safety disabled, motors are ready but not spinning yet

    AUTO_TAKEOFF, // Autonomous takeoff (PID controller active)
    FLYING,       // In-flight (manual or autonomous control)
    AUTO_LANDING, // Autonomous landing procedure in progress

    EMERGENCY_LANDING, // Emergency: low battery, lost signal, or critical failure
    DEGRADED,          // A non-critical component failed; flight is still possible
    ERROR              // Critical failure; flight is not allowed
};

#endif /* ESP32_DRONE_DRONE_STATE_H */