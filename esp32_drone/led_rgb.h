#ifndef ESP32_DRONE_LED_RGB_H
#define ESP32_DRONE_LED_RGB_H

#include <Arduino.h>

#ifdef RGB_BRIGHTNESS
#undef RGB_BRIGHTNESS
#endif
#define RGB_BRIGHTNESS 255

#ifndef RGB_BUITLIN
#define RGB_BUILTIN LED_BUILTIN
#endif

#define ON true
#define OFF false

enum class Color
{
    WHITE,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    PURPLE
};

void led_rgb_init(int led_rgb = RGB_BUILTIN);
void led_rgb_set(uint8_t r, uint8_t g, uint8_t b, bool state);
void led_rgb_blink(Color color, uint32_t interval_ms, uint32_t current_ms);

#define led_rgb_unset() led_rgb_set(0, 0, 0, OFF)
#define led_rgb_white(state) led_rgb_set(RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS, state)
#define led_rgb_red(state) led_rgb_set(RGB_BRIGHTNESS, 0, 0, state)
#define led_rgb_green(state) led_rgb_set(0, RGB_BRIGHTNESS, 0, state)
#define led_rgb_blue(state) led_rgb_set(0, 0, RGB_BRIGHTNESS, state)
#define led_rgb_yellow(state) led_rgb_set(RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0, state)
#define led_rgb_purple(state) led_rgb_set(RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS, state)

#endif /* ESP32_DRONE_LED_RGB_H */