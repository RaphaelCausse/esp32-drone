#include <Wire.h>
#include "mpu6050.h"
#include "vl53l0x.h"
#include "logger.h"

#define LED LED_BUILTIN

#define I2C_SDA 8
#define I2C_SCL 9

static const char *TAG = "main";

bool mpu_ready = false;

void setup()
{
  logger.begin();
  Wire.begin(I2C_SDA, I2C_SCL);

  mpu_ready = mpu6050_init();

  lox_ready = vl53l0x_init();
  
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
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
  
  if(lox_ready)
  {
    vl53l0x_poll(); //todo : faire la led qui s'allume quand on est trop proche
    // if(vl530x.distance != null){
    //   vl53l0x_LED(distance);
    // }
  }
  delay(200);
}