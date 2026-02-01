#ifndef UPDATE_GS_H
#define UPDATE_GS_H

#define TXD_PIN 32
#define RXD_PIN 35
#define RX_BUF_SIZE 1024
#define BAUD_RATE 115200

void UPDATE_GS(void *arg);
int sendData(int size, const char* data);
void uart_init(void);

#endif // UPDATE_GS_H

