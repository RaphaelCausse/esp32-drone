#ifndef ESP32_DRONE_VL53L0X_H
#define ESP32_DRONE_VL53L0X_H

bool vl53l0x_init(void);

void vl53l0x_poll(void);

void vl53l0x_LED(uint16_t distance)

#endif /* ESP32_DRONE_VL53L0X_H */