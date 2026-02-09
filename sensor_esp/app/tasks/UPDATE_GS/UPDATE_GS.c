#include <stdio.h>
#include "UPDATE_GS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataPacket.h"
#include "driver/uart.h"

const static char *TAG = "UPDATE_GS Task";
uint8_t msg_buffer[68] = {0}; // general buffer to receive slow lane messages

// -------------------- TASK LOOP -------------------- //
void UPDATE_GS(void *arg)
{
    ESP_LOGI(TAG, "Starting UPDATE_GS Task");
    uart_init();
    for(;;)
    {

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
    const int txBytes = uart_write_bytes(UART_NUM_1, data, size);
    return txBytes;
}

