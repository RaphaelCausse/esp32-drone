#ifndef ESP32_DRONE_TYPES_H
#define ESP32_DRONE_TYPES_H

#include <cstdint>
#include "constants.h"

/* Drone states */
enum state_drone_t
{
  STATE_DRONE_ERROR = -1,
  STATE_DRONE_OFF = 0,
  STATE_DRONE_READY,       /* Initialized, ready to receive commands */
  STATE_DRONE_TAKEOFF,     /* Autonomous takeoff */
  STATE_DRONE_HOVERING,    /* Stabilized hovering */
  STATE_DRONE_FLYING,      /* Flying */
  STATE_DRONE_STABILIZING, /* Trying to stabilize */
  STATE_DRONE_LANDING,     /* Autonomous landing */
  STATE_DRONE_EMERGENCY,   /* Emergency stop, landing */
  COUNT_STATE_DRONE,
};

/* Sensor states */
enum state_sensor_t
{
  STATE_SENSOR_OFF = 0,
  STATE_SENSOR_ACTIVE,  /* Initialized and active */
  STATE_SENSOR_DEAD,    /* Not initialized or inactive */
  COUNT_STATE_SENSOR,
};

/* Data from MPU6050 sensor */
struct data_mpu_t
{
  float accel_x; /* m/s^2 */
  float accel_y; /* m/s^2 */
  float accel_z; /* m/s^2 */
  float gyro_x;  /* rad/s */
  float gyro_y;  /* rad/s */
  float gyro_z;  /* rad/s */
};

/* Data from VL53L0X sensor */
struct data_vlx_t
{
  uint16_t distance; /* millimeters */
};

#endif /* ESP32_DRONE_TYPES_H */