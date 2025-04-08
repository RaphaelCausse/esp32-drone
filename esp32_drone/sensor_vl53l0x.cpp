#include "sensor_vl53l0x.h"
#include "logger.h"

static const char *TAG = "VL53L0X";

SensorVL53L0X::SensorVL53L0X() : m_state(SensorState::INACTIVE) {}

SensorVL53L0X::~SensorVL53L0X() {}

bool SensorVL53L0X::init(TwoWire *wire)
{
    int retries = 0;

    /*** Initialize sensor ***/

    m_i2c = wire;
    while (!m_vl53l0x.begin(VL53L0X_I2C_ADDR, false, m_i2c) && retries < VL53L0X_INIT_RETRY)
    {
        retries++;
        logger.warning(TAG, "Sensor %s not detected, retry %d...", TAG, retries);
        delay(200);
    }
    if (retries >= VL53L0X_INIT_RETRY)
    {
        m_state = SensorState::ERROR;
        logger.error(TAG, "Failed to initialize sensor %s", TAG);
        return false;
    }
    logger.info(TAG, "Sensor %s detected, I2C address 0x%x", TAG, VL53L0X_I2C_ADDR);

    /*** Configure sensor ***/

    m_vl53l0x.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    logger.info(TAG, "Sensor %s configured and active", TAG);

    m_state = SensorState::ACTIVE;

    return true;
}

void SensorVL53L0X::update()
{
    // Check sensor state before reading
    if (m_state != SensorState::ACTIVE)
    {
        logger.error(TAG, "Failed to read sensor %s, sensor is not active", TAG);
        return;
    }

    if (!read_distance())
    {
        m_state = SensorState::ERROR;
    }
}

bool SensorVL53L0X::read_distance()
{
    VL53L0X_RangingMeasurementData_t measure;
    VL53L0X_Error err = m_vl53l0x.rangingTest(&measure, false);

    if (err == VL53L0X_ERROR_NONE)
    {
        if (measure.RangeStatus == 0)
        {
            m_distance_cm = measure.RangeMilliMeter / 10.0f;
        }
        else
        {
            m_distance_cm = -1.0f;
            logger.warning(TAG, "Out of range or invalid measurement. Status: %d", measure.RangeStatus);
        }
    }
    else
    {
        m_distance_cm = -1.0f;
        logger.error(TAG, "Failed to read distance: %d", err);
        return false;
    }

    return true;
}

SensorState SensorVL53L0X::state() const { return m_state; }

bool SensorVL53L0X::is_active() const { return m_state == SensorState::ACTIVE; }

float SensorVL53L0X::distance_cm() const { return m_distance_cm; }