#include "mpu6050.h"
#include "vl53l0x.h"
#include "logger.h"

#define LED LED_BUILTIN

void setup()
{
  logger.begin();

  mpu6050_init();
  vl53l0x_init();
  
  pinMode(LED, OUTPUT);
}

void loop()
{
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
}
