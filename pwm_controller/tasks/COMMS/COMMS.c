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
#define DEBUG

#ifdef DEBUG
const static char *TAG = "COMMS";
#endif
static system_status_t sys_stat = {STATUS_UNINITIALIZED};
static sensor_data_t sensor_data = {0};

void COMMS(void *args)
{
    QueueHandle_t packet_queue = (QueueHandle_t)args;
    uint8_t next_response_address = STATUS_ADDRESS;
    packet_t rx_packet = {0};

    // loop until spi works is initialized
    while(spi3_slave_init() != ESP_OK){
        update_spi_bus_status(STATUS_ERROR);
        #ifdef DEBUG
        ESP_LOGI(TAG, "SPI BUS: Not initialized");
        #endif
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    esp_err_t transfer_status; 
    for(;;){
        if (next_response_address == DATA_ADDRESS) {
            get_sensor_data(&sensor_data);
            transfer_status = transfer_packet(
                sizeof(sensor_data_t),
                DATA_ADDRESS,
                (uint8_t*)&sensor_data,
                packet_queue,
                &rx_packet
            );
        } else {
            get_system_status(&sys_stat);
            transfer_status = transfer_packet(
                sizeof(system_status_t),
                STATUS_ADDRESS,
                (uint8_t*)&sys_stat,
                packet_queue,
                &rx_packet
            );
        }

        if (transfer_status == ESP_OK) {
            update_spi_bus_status(STATUS_OK);
            if (rx_packet.device_address == STATUS_ADDRESS || rx_packet.device_address == DATA_ADDRESS) {
                next_response_address = rx_packet.device_address;
            }
        } else if (transfer_status == ESP_ERR_TIMEOUT) {
            update_spi_bus_status(STATUS_ERROR);
            #ifdef DEBUG
            ESP_LOGW(TAG, "SPI transfer timed out waiting for master");
            #endif
        } else if (transfer_status == ESP_ERR_INVALID_CRC) {
            update_spi_bus_status(STATUS_CRC_FAILED);
        } else {
            update_spi_bus_status(STATUS_ERROR);
            #ifdef DEBUG
            ESP_LOGW(TAG, "SPI transfer failed: %s", esp_err_to_name(transfer_status));
            #endif
        }
    }
}
