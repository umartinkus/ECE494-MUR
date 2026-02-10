#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"

// tasks
#include "UPDATE_GS.h"

#define CORE0 0
#define CORE1 1
#define TASK_PRIO_4 4



void app_main(void)
{
    xTaskCreatePinnedToCore(UPDATE_GS, "UPDATE_GS_TASK_C0", 4096, NULL, TASK_PRIO_4, NULL, CORE1);
}
