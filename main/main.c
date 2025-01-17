#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

const char *TAG = "drone";

void app_main(void)
{
    const TickType_t delay = 1000 / portTICK_PERIOD_MS;

    while (1)
    {
        ESP_LOGI(TAG, "Hello world !");
        vTaskDelay(delay);
    }
}
