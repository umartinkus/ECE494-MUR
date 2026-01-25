#include <stdio.h>
#include "UPDATE_GS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataPacket.h"
#include "driver/uart.h"
// #include "driver/gpio.h"

const static char *TAG = "UPDATE_GS Task";

__uint8_t imu_buffer[22];
// -------------------- TASK LOOP -------------------- //
void UPDATE_GS(void *arg)
{
    dataBuffers_t* data_buffers = (dataBuffers_t*) arg;
    MessageBufferHandle_t pose_msg_buffer = (MessageBufferHandle_t)(data_buffers->pose_buff);
    if(!uart_is_driver_installed(UART_NUM_1)){init();}
    // MessageBufferHandle_t depth_msg_buffer = (MessageBufferHandle_t)(data_buffers->depth_buff);
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
        xMessageBufferReceive(pose_msg_buffer, imu_buffer, 22, portMAX_DELAY);

        // xMessageBufferReceive(depth_msg_buffer, buffer, 6, portMAX_DELAY);
        // ESP_LOGI(TAG, "Size of data: %d, address: %x", buffer[0], buffer[1]);
    }
}

void RECEIVE_GS(void *arg)
{
    if(!uart_is_driver_installed(UART_NUM_1)){init();}
}


void init(void)
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
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    return txBytes;
}

