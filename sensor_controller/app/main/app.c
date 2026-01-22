#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "GET_POSE.h"

#define CORE0 0
#define TASK_PRIO_3 3

void app_main(void)
{
    int task_id0 = 0;
    xTaskCreatePinnedToCore(GET_POSE, "pinned_task0_core0", 4096, (void*)task_id0, TASK_PRIO_3, NULL, CORE0);
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
