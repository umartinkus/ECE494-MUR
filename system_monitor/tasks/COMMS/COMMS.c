#include <stdio.h>
#include "COMMS.h"
#include "freertos/idf_additions.h"
#include "packet.h"
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
        uint8_t test_data[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        transfer_packet(10, 1, test_data, packet_queue);
    }
}
