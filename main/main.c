/******************************************************************************
 * \file main.c
 * \brief
 * \author Raphael CAUSSE - Melvyn MUNOZ
 *****************************************************************************/

/***** Includes **************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/***** Definitions ***********************************************************/

/***** Macros ****************************************************************/

/***** Global Variables ******************************************************/

/***** Static Variables ******************************************************/

static const char *tag = "main";

/***** Static Functions Prototypes *******************************************/

/***** Functions *************************************************************/

void app_main(void)
{
    ESP_LOGI(tag, "Hello from ESP32-S3 !");

    const TickType_t delay_ticks = pdMS_TO_TICKS(1000);

    for (;;)
    {
        ESP_LOGI(tag, "ticks since scheduler started: %lu", xTaskGetTickCount());
        vTaskDelay(delay_ticks);
    }
}

/***** Static Functions Definitions ******************************************/