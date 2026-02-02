#include <stdio.h>
#include "UART_LISTEN.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#define BUF_SIZE 256

void UART_LISTEN(void* params) {
    QueueHandle_t uart_queue = (QueueHandle_t)params;

    uint8_t uart_read_buffer[BUF_SIZE];
    size_t available_bytes;
    size_t bytes_read;

    // enter the main listening loop
    for (;;) {
        // get available bytes
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, &available_bytes));
        
        // if no available bytes, block for 10ms and try again
        if (!available_bytes) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // if more than buf size available, empty buf size
        if (available_bytes > (size_t)BUF_SIZE) {
            available_bytes = BUF_SIZE;
        }

        // read into the buffer
        bytes_read = uart_read_bytes(UART_NUM_1, uart_read_buffer, available_bytes, pdMS_TO_TICKS(10));

        // push bytes into the buffer
        for (int i = 0; i < bytes_read; i++) {
            xQueueSendToBack(uart_queue, &uart_read_buffer[i], pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


