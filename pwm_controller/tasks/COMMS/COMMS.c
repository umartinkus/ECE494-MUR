#include <stdint.h>
#include <stdio.h>
#include "COMMS.h"
#include "common_types.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "packet.h"
#include "spi.h"
#include "configuration.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "sys_common.h"
#include "esp_log.h"
#include "freertos/task.h"

#define STATUS_ADDRESS 0

const static char *TAG = "COMMS";
static system_status_t *sys_stat = {0};


void COMMS(void *args)
{
    QueueHandle_t packet_queue = (QueueHandle_t)args;

    // loop until spi works is initialized
    while(spi3_slave_init() != ESP_OK){
        get_system_status(sys_stat);
        sys_stat->spi_bus_status = STATUS_ERROR;
        update_system_status(*sys_stat);

        ESP_LOGI(TAG, "SPI BUS: Not initialized");
        vTaskDelay(pdMS_TO_TICKS(200));
    }


    esp_err_t transfer_status; 
    for(;;){
        get_system_status(sys_stat); // accessing the global system state and copying it by value into sys_stat
        transfer_status = transfer_packet(sizeof(system_status_t), STATUS_ADDRESS, (uint8_t*)sys_stat, packet_queue);
        
        // check if the recieval is good or not
        if (transfer_status == ESP_ERR_INVALID_CRC) {
            sys_stat->spi_bus_status = STATUS_CRC_FAILED;
        }
        // just going to ignore packets without sync bytes for now

        update_system_status(*sys_stat);
        vTaskDelay(pdMS_TO_TICKS(TRANSFER_PERIOD));
    }
}
