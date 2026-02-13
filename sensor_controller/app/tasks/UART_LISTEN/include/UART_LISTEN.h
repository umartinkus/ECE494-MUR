#ifndef UART_LISTEN_H
#define UART_LISTEN_H
#include "freertos/FreeRTOS.h"

// this struct exists to pass multiple queues into the uart listen task
struct UartVariables {
    QueueHandle_t* uart_queue;
    QueueHandle_t* parsed_queue;
    uint8_t test;
};

void UART_LISTEN(void* params);

#endif
