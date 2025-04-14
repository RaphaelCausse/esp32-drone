#include <Adafruit_Sensor.h>
#include "sensor_mpu6050.h"
#include "logger.h"

static const char *TAG = "MPU6050";

SensorMPU6050::SensorMPU6050() : m_state(SensorState::INACTIVE) {}

SensorMPU6050::~SensorMPU6050() {}

bool SensorMPU6050::init(TwoWire *wire)
{
    int retries = 0;

    /*** Initialize sensor ***/

    m_i2c = wire;
    while (!m_mpu6050.begin(MPU6050_I2CADDR_DEFAULT, m_i2c) && retries < MPU6050_INIT_RETRY)
    {
        retries++;
        logger.warning(TAG, "Sensor %s not detected, retry %d...", TAG, retries);
        delay(200);
    }
    if (retries >= MPU6050_INIT_RETRY)
    {
        m_state = SensorState::ERROR;
        logger.error(TAG, "Failed to initialize sensor %s", TAG);
        return false;
    }
    logger.info(TAG, "Sensor %s detected, I2C address 0x%x", TAG, MPU6050_I2CADDR_DEFAULT);

    /*** Configure sensor ***/

    m_mpu6050.setAccelerometerRange(MPU6050_RANGE_4_G);
    m_mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    m_mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
    m_mpu6050.setTemperatureStandby(true); // Disable temperature sensor
    logger.info(TAG, "Sensor %s configured and active", TAG);

    m_state = SensorState::ACTIVE;

    return true;
}

bool SensorMPU6050::calibrate()
{
    const int calibration_samples = 2000;
    sensors_event_t g;

    // Check sensor state
    if (m_state != SensorState::ACTIVE)
    {
        logger.error(TAG, "Failed to calibrate, sensor %s is not active", TAG);
        return false;
    }

    // Reset calibration values
    m_roll_calibration = 0;
    m_pitch_calibration = 0;
    m_yaw_calibration = 0;

    // Gyroscope calibration
    logger.info(TAG, "Starting sensor %s calibration...", TAG);
    for (int i = 0; i < calibration_samples; i++)
    {
        if (!m_mpu6050.getGyroSensor()->getEvent(&g))
        {
            logger.error(TAG, "Failed to calibrate, sensor %s is not responding", TAG);
            return false;
        }
        m_roll_calibration += g.gyro.x;
        m_pitch_calibration += g.gyro.y;
        m_yaw_calibration += g.gyro.z;
        delay(1);
    }
    m_roll_calibration /= calibration_samples;
    m_pitch_calibration /= calibration_samples;
    m_yaw_calibration /= calibration_samples;
    logger.info(TAG, "Calibration finished");

    return true;
}

void SensorMPU6050::update()
{
    // Check sensor state before reading
    if (m_state != SensorState::ACTIVE)
    {
        logger.warning(TAG, "Failed to read, sensor %s is not active", TAG);
        return;
    }

    // Get new sensor readings
    if (!read_gyro())
    {
        m_state = SensorState::ERROR;
        return;
    }
    if (!read_accel())
    {
        m_state = SensorState::ERROR;
        return;
    }
}

bool SensorMPU6050::check()
{
    sensors_event_t g;

    // Check sensor state before reading
    if (m_state != SensorState::ACTIVE)
    {
        logger.error(TAG, "Sensor %s is not active", TAG);
        return false;
    }

    // Test reading sensor
    if (!m_mpu6050.getGyroSensor()->getEvent(&g) ||
        (g.gyro.x == 0 && g.gyro.y == 0 && g.gyro.z == 0))
    {
        logger.error(TAG, "Sensor %s is not responding", TAG);
        m_state = SensorState::ERROR;
        return false;
    }

    return true;
}

bool SensorMPU6050::read_gyro()
{
    sensors_event_t g;

    // Read sensor data
    m_mpu6050.getGyroSensor()->getEvent(&g);
    if (g.gyro.x == 0 && g.gyro.y == 0 && g.gyro.z == 0)
    {
        logger.warning(TAG, "Failed to read gyroscope", TAG);
        return false;
    }

    // Apply calibration
    m_roll = g.gyro.x - m_roll_calibration;
    m_pitch = g.gyro.y - m_pitch_calibration;
    m_yaw = g.gyro.z - m_yaw_calibration;

    // DEBUG
    logger.debug(TAG, "Gyro (rad/s)\t| Roll: %.3f\t| Pitch: %.3f\t| Yaw: %.3f", m_roll, m_pitch, m_yaw);
    // END DEBUG

    return true;
}

bool SensorMPU6050::read_accel()
{
    sensors_event_t a;

    // Read sensor data
    m_mpu6050.getAccelerometerSensor()->getEvent(&a);
    if (a.acceleration.x == 0 && a.acceleration.y == 0 && a.acceleration.z == 0)
    {
        logger.warning(TAG, "Failed to read accelerometer", TAG);
        return false;
    }

    m_accel_x = a.acceleration.x;
    m_accel_y = a.acceleration.y;
    m_accel_z = a.acceleration.z;

    // DEBUG
    logger.debug(TAG, "Accel (m/s^2)\t| X: %.3f\t| Y: %.3f\t| Z: %.3f", m_accel_x, m_accel_y, m_accel_z);
    // END DEBUG

    return true;
}

SensorState SensorMPU6050::state() const { return m_state; }

bool SensorMPU6050::is_active() const { return m_state == SensorState::ACTIVE; }

float SensorMPU6050::roll() const { return m_roll; }

float SensorMPU6050::pitch() const { return m_pitch; }

float SensorMPU6050::yaw() const { return m_yaw; }
