#include <stdio.h>
#include "uart.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "hal/uart_types.h"

#define TXD         (34)
#define RXD         (35)

const uart_port_t uart_num = UART_NUM_0;
const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;

void config_uart(){
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
// Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

  ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 20, &uart_queue, 0));
}
