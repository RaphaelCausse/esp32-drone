#include <Adafruit_MPU6050.h>
#include "mpu6050.h"
#include "logger.h"

static const char *TAG = "mpu6050";

static Adafruit_MPU6050 mpu;
static Adafruit_Sensor *mpu_sensor_accel;
static Adafruit_Sensor *mpu_sensor_gyro;

bool mpu6050_init()
{
  /* Initialization */
  if (!mpu.begin())
  {
    logger.error(TAG, "Failed to find sensor %s", TAG);
    return false;
  }
  logger.info(TAG, "Found sensor %s at address 0x%x", TAG, MPU6050_I2CADDR_DEFAULT);

  /* Sensors configurations */
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /* Sensors informations */
  mpu_sensor_accel = mpu.getAccelerometerSensor();
  mpu_sensor_gyro = mpu.getGyroSensor();
  if (mpu_sensor_accel && mpu_sensor_gyro)
  {
    mpu_sensor_accel->printSensorDetails();
    mpu_sensor_gyro->printSensorDetails();
  }
  else
  {
    logger.warning(TAG, "Sensor details unavailable");
  }
  
  return true;
}

void mpu6050_reset()
{
  logger.info(TAG, "Resetting sensor %s...", TAG);
  Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(MPU6050_CMD_RESET);
  Wire.endTransmission();
  delay(100);
  
  // Restart sensor after reset
  if (mpu.begin())
  {
    logger.info(TAG, "Successfully restarted sensor %s after reset", TAG);
  }
  else
  {
    logger.error(TAG, "Failed to restart sensor %s after reset", TAG);
  }
}

void mpu6050_poll_accel(data_mpu_t *mpu_data)
{
  sensors_event_t accel;
  mpu_sensor_accel->getEvent(&accel);
  
  mpu_data->accel_x = accel.acceleration.x;
  mpu_data->accel_y = accel.acceleration.y;
  mpu_data->accel_z = accel.acceleration.z;
}

void mpu6050_poll_gyro(data_mpu_t *mpu_data)
{
  sensors_event_t gyro;
  mpu_sensor_gyro->getEvent(&gyro);

  mpu_data->gyro_x = gyro.gyro.x;
  mpu_data->gyro_y = gyro.gyro.y;
  mpu_data->gyro_z = gyro.gyro.z;
}

bool mpu6050_check(data_mpu_t *mpu_data)
{
  return !(mpu_data->accel_x == 0.0 &&
           mpu_data->accel_y == 0.0 &&
           mpu_data->accel_z == 0.0 &&
           mpu_data->gyro_x == 0.0 &&
           mpu_data->gyro_y == 0.0 &&
           mpu_data->gyro_z == 0.0);
}

void mpu6050_print(data_mpu_t *mpu_data)
{
  Serial.println("Acceleration (m/s2)");
  Serial.print("    X: ");
  Serial.println(mpu_data->accel_x);
  Serial.print("    Y: ");
  Serial.println(mpu_data->accel_y);
  Serial.print("    Z: ");
  Serial.println(mpu_data->accel_z);

  Serial.println("Gyroscopic (rad/s)");
  Serial.print("    X: ");
  Serial.println(mpu_data->gyro_x);
  Serial.print("    Y: ");
  Serial.println(mpu_data->gyro_y);
  Serial.print("    Z: ");
  Serial.println(mpu_data->gyro_z);
  Serial.println("");
}