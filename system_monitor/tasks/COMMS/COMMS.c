#include <stdio.h>
#include "COMMS.h"
#include "spi.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "sys_common.h"

const static char *TAG = "COMMS";
static system_status_t sys_stat;

void COMMS(void *args)
{
    // while(spi3_slave_init() != ESP_OK){

    // }
    for(;;){
        get_system_status(&sys_stat); // accessing the global system state and copying it by value into sys_stat
        ESP_LOGI(TAG, "LEAK1 STATUS: %s", error_code_to_string(sys_stat.leak1_status));
        ESP_LOGI(TAG,"LEAK2 STATUS: %s", error_code_to_string(sys_stat.leak2_status));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
