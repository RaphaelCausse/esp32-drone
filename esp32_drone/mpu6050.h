#ifndef ESP32_DRONE_MPU6050_H
#define ESP32_DRONE_MPU6050_H

struct mpu_data_t
{
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

bool mpu6050_init(void);

void mpu6050_poll(void);

void mpu6050_print(void);

#endif /* ESP32_DRONE_MPU6050_H */