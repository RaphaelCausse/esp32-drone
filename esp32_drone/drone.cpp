#include "drone.h"
#include "logger.h"
#include "mpu6050.h"
#include "vl53l0x.h"

static const char *TAG = "drone";

Drone::Drone()
{
  m_state_drone = STATE_DRONE_OFF;
  m_state_sensor_mpu = STATE_SENSOR_OFF;
  m_state_sensor_vlx = STATE_SENSOR_OFF;
}

bool Drone::init()
{
  if (!sensors_init())
  {
    return false;
  }

  return true;
}

bool Drone::sensors_init()
{
  uint16_t num_try = 0;

  /* Try to init MPU6050 sensor */
  do
  {
    if (mpu6050_init())
    {
      m_state_sensor_mpu = STATE_SENSOR_ACTIVE;
      logger.info(TAG, "Successfully initialized sensor mpu6050");
    }
    else
    {
      logger.warning(TAG, "Retrying (%u) to initialize sensor mpu6050...", num_try);
      num_try++;
      delay(100);
    }

    if (num_try >= SENSOR_INIT_MAX_TRY)
    {
      m_state_sensor_mpu = STATE_SENSOR_DEAD;
      logger.error(TAG, "Failed to initialize sensor mpu6050");
      break;
    }
  } while (m_state_sensor_mpu != STATE_SENSOR_ACTIVE);
  
  /* Try to init VL53L0X sensor */
  num_try = 0;
  do
  {
    if (vl53l0x_init())
    {
      m_state_sensor_vlx = STATE_SENSOR_ACTIVE;
      logger.info(TAG, "Successfully initialized sensor vl53l0x");
    }
    else
    {
      logger.warning(TAG, "Retrying (%u) to initialize sensor vl53l0x...", num_try);
      num_try++;
      delay(100);
    }

    if (num_try >= SENSOR_INIT_MAX_TRY)
    {
      m_state_sensor_vlx = STATE_SENSOR_DEAD;
      logger.error(TAG, "Failed to initialize sensor vl53l0x");
      break;
    }
  } while (m_state_sensor_vlx != STATE_SENSOR_ACTIVE);

  return (m_state_sensor_mpu == STATE_SENSOR_ACTIVE && m_state_sensor_vlx == STATE_SENSOR_ACTIVE);
}

void Drone::state_machine()
{
  uint16_t mpu_num_invalid = 0;

  switch (m_state_drone)
  {
  case STATE_DRONE_OFF:
    if (init())
    {
      m_state_drone = STATE_DRONE_READY;
    }
    break;

  case STATE_DRONE_READY:
    mpu6050_poll_accel(&m_data_mpu);
    mpu6050_poll_gyro(&m_data_mpu);
    mpu6050_print(&m_data_mpu);
    if (!mpu6050_check(&m_data_mpu))
    {
      mpu_num_invalid++;
    }
    if (mpu_num_invalid >= SENSOR_POLL_MAX_INVALID)
    {
      mpu_num_invalid = 0;
      mpu6050_reset();
    }
    break;

  default:
    break;
  }
  delay(200);
}
