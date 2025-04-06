#ifndef ESP32_DRONE_MPU6050_H
#define ESP32_DRONE_MPU6050_H

#include "types.h"

bool mpu6050_init();
void mpu6050_reset();

void mpu6050_poll_accel(data_mpu_t *mpu_data);
void mpu6050_poll_gyro(data_mpu_t *mpu_data);

bool mpu6050_check(data_mpu_t *mpu_data);

void mpu6050_print(data_mpu_t *mpu_data);

#endif /* ESP32_DRONE_MPU6050_H */