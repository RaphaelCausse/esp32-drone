#include "sensor_vl53l0x.h"
#include "logger.h"

static const char *TAG = "VL53L0X";

SensorVL53L0X::SensorVL53L0X() : m_state(SensorState::INACTIVE) {}

SensorVL53L0X::~SensorVL53L0X() {}

bool SensorVL53L0X::init(TwoWire *wire)
{
    int retries = 0;

    /* Initialize sensor */
    m_i2c = wire;
    while (!m_vl53l0x.begin(VL53L0X_I2C_ADDR, false, m_i2c) && retries < VL53L0X_INIT_RETRY)
    {
        retries++;
        logger.warning(TAG, "Sensor %s not detected, retry %d...", TAG, retries);
    }
    if (retries >= VL53L0X_INIT_RETRY)
    {
        m_state = SensorState::ERROR;
        logger.error(TAG, "Failed to initialize sensor %s", TAG);
        return false;
    }
    logger.info(TAG, "Sensor %s detected, I2C address 0x%x", TAG, VL53L0X_I2C_ADDR);

    /* Configure sensor */

    logger.info(TAG, "Sensor %s configured", TAG);
    return true;
}

SensorState SensorVL53L0X::state() const
{
    return m_state;
}