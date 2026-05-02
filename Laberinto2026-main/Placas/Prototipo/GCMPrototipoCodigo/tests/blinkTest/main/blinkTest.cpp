#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

void app_main(void)
{
    gpio_set_direction(GPIO_NUM_47, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(GPIO_NUM_47, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gpio_set_level(GPIO_NUM_47, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}