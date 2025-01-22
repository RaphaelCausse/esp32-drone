#include "mpu6050.h"
#include "logger.h"

static const char *TAG = "mpu6050";

bool mpu6050_init(void)
{
  logger.info(TAG, "Initializing mpu6050 device");
  return true;
}