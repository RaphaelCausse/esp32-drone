#include <Arduino.h>
#include <Wire.h>
#include "logger.h"
#include "ledRGB.h"
#include "drone.h"

#define LED LED_BUILTIN

static const char *TAG = "main";

Drone drone;

void setup()
{
  /* Init Serial and logger */
  logger.begin(LOGGER_BAUDRATE);

  /* Init I2C */
  if (Wire.begin(GPIO_I2C_SDA, GPIO_I2C_SCL))
  {
    logger.info(TAG, "Successfully initialized I2C on SDA: %d, SCL: %d", GPIO_I2C_SDA, GPIO_I2C_SCL);
  }
  else
  {
    logger.error(TAG, "Failed to initialize I2C on SDA: %d, SCL: %d", GPIO_I2C_SDA, GPIO_I2C_SCL);
    /* TODO: Should be error state */
  }
}

void loop()
{
  drone.state_machine();
}