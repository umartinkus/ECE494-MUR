#include <stdio.h>
#include "COMMS.h"
#include "freertos/idf_additions.h"
#include "spi.h"
#include "configuration.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

void COMMS(void *args)
{
    QueueHandle_t packet_queue = (QueueHandle_t)args;
    static const char *TAG = "COMMS";

    if (spi3_slave_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI3 slave");
        vTaskDelete(NULL);
    }

    for (;;) {
        uint8_t tx_buf[PACKET_SIZE] = {85, 2, 3, 4};
        uint8_t rx_buf[PACKET_SIZE] = {0};

        ESP_LOGI(TAG, "Waiting for SPI transaction");

        esp_err_t ret = spi_transaction(tx_buf, rx_buf, sizeof(tx_buf));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        }
    }
}
