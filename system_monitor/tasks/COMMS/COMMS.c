#include <stdio.h>
#include "COMMS.h"
#include "freertos/idf_additions.h"
#include "packet.h"
#include "spi.h"
#include "configuration.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "sys_common.h"
#include "esp_log.h"
#include "freertos/task.h"

const static char *TAG = "COMMS";
static system_status_t sys_stat;


void COMMS(void *args)
{
    QueueHandle_t packet_queue = (QueueHandle_t)args;
    static const char *TAG = "COMMS";

    if (spi3_slave_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI3 slave");
        vTaskDelete(NULL);
    }

    // while(spi3_slave_init() != ESP_OK){
    // update the system status
    // }
    for(;;){
        get_system_status(&sys_stat); // accessing the global system state and copying it by value into sys_stat
        ESP_LOGI(TAG, "LEAK1 STATUS: %s", error_code_to_string(sys_stat.leak1_status));
        ESP_LOGI(TAG,"LEAK2 STATUS: %s", error_code_to_string(sys_stat.leak2_status));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
