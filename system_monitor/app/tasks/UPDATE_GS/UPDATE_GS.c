#include <stdio.h>
#include "UPDATE_GS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataPacket.h"
#include "driver/uart.h"
#include "ms5837.h"

const static char *TAG = "UPDATE_GS Task";
uint8_t msg_buffer[68] = {0}; // buffer to hold data packet 

// Private variables
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t bar30_handle;

// -------------------- TASK LOOP -------------------- //
void UPDATE_GS(void *arg)
{
    QueueHandle_t data_queue = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "Starting UPDATE_GS Task");
    uart_init();
    i2c_master_init(&bus_handle, &bar30_handle);
    bar30_setup(bus_handle, bar30_handle);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Give some time for the sensor to initialize before starting the loop
    for(;;)
    {
        // 1. Read sensor data and fill the data packet
        // 2. Serialize the data packet into msg_buffer
        // 3. Send the data over UART
        // 4. Delay for the desired update rate (e.g., 2000ms for 0.5Hz)

        sendData(68, (const char*)msg_buffer);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2Do: adjust this delay as needed
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

