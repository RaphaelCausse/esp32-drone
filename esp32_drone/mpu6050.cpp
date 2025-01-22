#include <Adafruit_MPU6050.h>
#include "mpu6050.h"
#include "logger.h"

static const char *TAG = "mpu6050";

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;
Adafruit_Sensor *mpu_gyro;
mpu_data_t mpu_data;

bool mpu6050_init(void)
{
  if (!mpu.begin())
  {
    logger.error(TAG, "Failed to initialize %s", TAG);
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();

  logger.info(TAG, "Initialized %s successfully", TAG);
  mpu_accel->printSensorDetails();
  mpu_gyro->printSensorDetails();
  
  return true;
}

void mpu6050_poll(void)
{
  sensors_event_t accel;
  sensors_event_t gyro;

  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  mpu_data.accel_x = accel.acceleration.x;
  mpu_data.accel_y = accel.acceleration.y;
  mpu_data.accel_z = accel.acceleration.z;
  mpu_data.gyro_x = gyro.gyro.x;
  mpu_data.gyro_y = gyro.gyro.y;
  mpu_data.gyro_z = gyro.gyro.z;
}

void mpu6050_print(void)
{
  Serial.println("Acceleration (m/s2)");
  Serial.print("    X: ");
  Serial.println(mpu_data.accel_x);
  Serial.print("    Y: ");
  Serial.println(mpu_data.accel_y);
  Serial.print("    Z: ");
  Serial.println(mpu_data.accel_z);

  Serial.println("Gyroscopic (rad/s)");
  Serial.print("    X: ");
  Serial.println(mpu_data.gyro_x);
  Serial.print("    Y: ");
  Serial.println(mpu_data.gyro_y);
  Serial.print("    Z: ");
  Serial.println(mpu_data.gyro_z);
  Serial.println("");
}