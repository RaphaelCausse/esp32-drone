#include "led_rgb.h"

static uint32_t last_blink_ms = 0; // Last blink time in milliseconds
static bool led_state = false;     // Current LED state

void led_rgb_init(int led_rgb)
{
    pinMode(led_rgb, OUTPUT);
    led_rgb_unset();
}

void led_rgb_set(uint8_t r, uint8_t g, uint8_t b, bool state)
{
    led_state = state;
    if (led_state)
    {
        rgbLedWrite(RGB_BUILTIN, r, g, b);
    }
    else
    {
        rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    }
}

void led_rgb_blink(Color color, uint32_t interval_ms, uint32_t current_ms)
{
    if (current_ms - last_blink_ms >= interval_ms)
    {
        led_state = !led_state;

        switch (color)
        {
        case Color::WHITE:
            led_rgb_white(led_state);
            break;

        case Color::RED:
            led_rgb_red(led_state);
            break;

        case Color::GREEN:
            led_rgb_green(led_state);
            break;

        case Color::BLUE:
            led_rgb_blue(led_state);
            break;

        case Color::YELLOW:
            led_rgb_yellow(led_state);
            break;

        case Color::PURPLE:
            led_rgb_purple(led_state);
            break;
        }

        last_blink_ms = current_ms;
    }
}
