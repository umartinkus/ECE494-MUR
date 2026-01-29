#include <stdio.h>
#include "UPDATE_GS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataPacket.h"
#include "driver/uart.h"

const static char *TAG = "UPDATE_GS Task";
uint8_t msg_buffer[68]; // general buffer to receive slow lane messages

// -------------------- TASK LOOP -------------------- //
void UPDATE_GS(void *arg)
{
    ESP_LOGI(TAG, "Starting UPDATE_GS Task");
    MessageBufferHandle_t slow_lane_buffer = (MessageBufferHandle_t) arg;
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
        xMessageBufferReceive(slow_lane_buffer, msg_buffer, 68, portMAX_DELAY);
        uint8_t dataSize = msg_buffer[2];
        void * data_ptr = malloc(dataSize + 4); // allocate memory for incoming data
        if (data_ptr == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed");
            continue; // Skip this iteration if memory allocation fails
        }
        memcpy(data_ptr, msg_buffer, dataSize + 4); // copy
        sendData(dataSize + 4, (const char*)data_ptr); // send data over UART
        free(data_ptr); // free allocated memory
    }
}

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);    
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int sendData(int size, const char* data)
{
    ESP_LOGI(TAG, "Data to send:");
    for (int i = 0; i < size; i++) {
        ESP_LOGI(TAG, "%02X ", (unsigned char)data[i]);
    }
    ESP_LOGI(TAG, "Sending %d bytes over UART", size);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, size);
    ESP_LOGI(TAG, "Sent %d bytes over UART", txBytes);
    return txBytes;
}

