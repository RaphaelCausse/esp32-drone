#include "sensor_mpu6050.h"
#include "logger.h"

static const char *TAG = "MPU6050";

SensorMPU6050::SensorMPU6050() : m_state(SensorState::INACTIVE) {}

SensorMPU6050::~SensorMPU6050() {}

bool SensorMPU6050::init(TwoWire *wire)
{
    int retries = 0;

    /* Initialize sensor */
    m_i2c = wire;
    while (!m_mpu6050.begin(MPU6050_I2CADDR_DEFAULT, m_i2c) && retries < MPU6050_INIT_RETRY)
    {
        retries++;
        logger.warning(TAG, "Sensor %s not detected, retry %d...", TAG, retries);
    }
    if (retries >= MPU6050_INIT_RETRY)
    {
        m_state = SensorState::ERROR;
        logger.error(TAG, "Failed to initialize sensor %s", TAG);
        return false;
    }
    logger.info(TAG, "Sensor %s detected, I2C address 0x%x", TAG, MPU6050_I2CADDR_DEFAULT);

    /* Configure sensor */
    m_mpu6050.setAccelerometerRange(MPU6050_RANGE_4_G);
    m_mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    m_mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
    m_state = SensorState::ACTIVE;

    logger.info(TAG, "Sensor %s configured", TAG);
    return true;
}

void SensorMPU6050::calibrate()
{
    delay(2000);
}

SensorState SensorMPU6050::state() const
{
    return m_state;
}
