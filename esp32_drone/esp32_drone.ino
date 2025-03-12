#include <Wire.h>
#include "mpu6050.h"
#include "vl53l0x.h"
#include "logger.h"

#define I2C_SDA 8
#define I2C_SCL 9

static const char *TAG = "main";

bool mpu_ready = false;

void setup()
{
  logger.begin();
  Wire.begin(I2C_SDA, I2C_SCL);

  mpu_ready = mpu6050_init();
}

void loop()
{
  if (mpu_ready)
  {
    mpu6050_poll();
    mpu6050_print();
  }
  else
  {
    logger.warning(TAG, "Cannot read from mpu6050");
  }

  delay(200);
}