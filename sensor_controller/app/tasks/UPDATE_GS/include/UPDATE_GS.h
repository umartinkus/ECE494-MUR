#ifndef UPDATE_GS_H
#define UPDATE_GS_H

#include "driver/uart.h"

// if you change this to the usb, make sure to change all 3
#define UART_PORT UART_NUM_1
#define TXD_PIN 21
#define RXD_PIN 19

#define RX_BUF_SIZE 1024
#define BAUD_RATE 115200

void UPDATE_GS(void *arg);
int sendData(int size, const char* data);
void uart_init(void);

#endif // UPDATE_GS_H
