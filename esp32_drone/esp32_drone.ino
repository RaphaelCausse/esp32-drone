#include "mpu6050.h"
#include "vl53l0x.h"

#define LED LED_BUILTIN

void setup()
{
  Serial.begin(115200);

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
