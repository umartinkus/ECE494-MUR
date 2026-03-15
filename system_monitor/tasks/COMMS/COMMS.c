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
#define DATA_ADDRESS 1

#ifdef DEBUG
const static char *TAG = "COMMS";
#endif
static system_status_t sys_stat = {STATUS_UNINITIALIZED};
static sensor_data_t sensor_data = {0};

void COMMS(void *args)
{
    QueueHandle_t packet_queue = (QueueHandle_t)args;

    // loop until spi works is initialized
    while(spi3_slave_init() != ESP_OK){
        get_system_status(&sys_stat);
        sys_stat.spi_bus_status = STATUS_ERROR;
        update_system_status(sys_stat);
        #ifdef DEBUG
        ESP_LOGI(TAG, "SPI BUS: Not initialized");
        #endif
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    esp_err_t transfer_status; 
    for(;;){
        /* 1. Read system status and transmit over SPI*/
        get_system_status(&sys_stat); // accessing the global system state and copying it by value
        transfer_status = transfer_packet(sizeof(system_status_t), STATUS_ADDRESS, (uint8_t*)&sys_stat, packet_queue);
        if (transfer_status == ESP_OK) {
            sys_stat.spi_bus_status = STATUS_OK;
        } else if (transfer_status == ESP_ERR_TIMEOUT) {
            sys_stat.spi_bus_status = STATUS_ERROR;
            #ifdef DEBUG
            ESP_LOGW(TAG, "SPI transfer timed out waiting for master");
            #endif
        } else if (transfer_status == ESP_ERR_INVALID_CRC) {
            sys_stat.spi_bus_status = STATUS_CRC_FAILED;
        } else {
            sys_stat.spi_bus_status = STATUS_ERROR;
            #ifdef DEBUG
            ESP_LOGW(TAG, "SPI transfer failed: %s", esp_err_to_name(transfer_status));
            #endif
        }
        /* 2. Read sensor data and transmit over SPI*/
        get_sensor_data(&sensor_data);
        transfer_status = transfer_packet(sizeof(sensor_data_t), DATA_ADDRESS, (uint8_t*)&sensor_data, packet_queue);
        if (transfer_status == ESP_OK) {
            sys_stat.spi_bus_status = STATUS_OK;
        } else if (transfer_status == ESP_ERR_TIMEOUT) {
            sys_stat.spi_bus_status = STATUS_ERROR;
            #ifdef DEBUG
            ESP_LOGW(TAG, "SPI transfer timed out waiting for master");
            #endif
        } else if (transfer_status == ESP_ERR_INVALID_CRC) {
            sys_stat.spi_bus_status = STATUS_CRC_FAILED;
        } else {
            sys_stat.spi_bus_status = STATUS_ERROR;
            #ifdef DEBUG
            ESP_LOGW(TAG, "SPI transfer failed: %s", esp_err_to_name(transfer_status));
            #endif
        }
        
        /* 3. Update global system status*/
        update_system_status(sys_stat);
        vTaskDelay(pdMS_TO_TICKS(TRANSFER_PERIOD));
    }
}
