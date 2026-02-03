// std lib includes
#include <stdio.h>

// custom package includes
#include "UART_LISTEN.h"
#include "driver/uart.h"
#include "state_machine.h"
#include "UPDATE_GS.h"

// esp idf includes
#include "esp_err.h"
#include "esp_log.h"

// freertos includes
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#define BUF_SIZE 256

extern State state; // gonna be so real, this is kinda dumb but it needs to be defined here

void UART_LISTEN(void* params) {
    // get the pvParams struct pointer
    struct UartVariables* pvParams = (struct UartVariables*)params;

    // get the queue handles from the struct pointer
    QueueHandle_t uart_queue = *pvParams->uart_queue;
    QueueHandle_t parsed_queue = *pvParams->parsed_queue;

    // setting some variables to be used later
    uint8_t uart_read_buffer[BUF_SIZE];
    size_t bytes_read;
    size_t available_bytes;

    state = sync_state;

    // enter the main listening loop
    for (;;) {
        // get available bytes
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, &available_bytes));
        
        // if no available bytes, block for 10ms and try again
        if (!available_bytes) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // if more than buf size available, empty only buf size worth of bytes
        if (available_bytes > (size_t)BUF_SIZE) {
            available_bytes = BUF_SIZE;
        }

        // read into the buffer
        bytes_read = uart_read_bytes(UART_PORT, uart_read_buffer, available_bytes, pdMS_TO_TICKS(10));

        // push bytes into the buffer
        for (int i = 0; i < bytes_read; i++) {
            state(uart_read_buffer[i], uart_queue);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


