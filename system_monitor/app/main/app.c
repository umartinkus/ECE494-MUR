#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "dataPacket.h"

// Tasks
#include "UPDATE_GS.h"
#define CORE0 0
#define CORE1 1
#define TASK_PRIO_4 4

// Global Buffers
QueueHandle_t data_queue; // Queue for sending data from sensor reading task to UART task

void app_main(void)
{
    data_queue = xQueueCreate(10, sizeof(uartPacket_t)); // Create a queue to hold uartPacket_t structures
    if(data_queue == NULL) {
        ESP_LOGE("APP_MAIN", "Failed to create data_queue");
        return;
    }
    xTaskCreatePinnedToCore(UPDATE_GS, "UPDATE_GS_TASK_C0", 4096, (void*)data_queue, TASK_PRIO_4, NULL, CORE1);
}
