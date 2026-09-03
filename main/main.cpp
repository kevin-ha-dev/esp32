#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"

extern "C" void app_main(void)
{
    motor_init();
    printf("Forward only\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    motor_set(0.5f);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
