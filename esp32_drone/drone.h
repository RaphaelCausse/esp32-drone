#ifndef ESP32_DRONE_DRONE_H
#define ESP32_DRONE_DRONE_H

#include "types.h"

class Drone
{
public:
  Drone();

  bool init();
  bool sensors_init();
  void state_machine();

private:
  state_drone_t  m_state_drone;
  state_sensor_t m_state_sensor_mpu;
  data_mpu_t     m_data_mpu;
  state_sensor_t m_state_sensor_vlx;
  data_vlx_t     m_data_vlx;
};

#endif /* ESP32_DRONE_DRONE_H */