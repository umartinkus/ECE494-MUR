#ifndef UPDATE_GS_H
#define UPDATE_GS_H

#define TXD_PIN 1
#define RXD_PIN 3
#define RX_BUF_SIZE 1024
#define BAUD_RATE 115200

void UPDATE_GS(void *arg);
int sendData(const char* logName, const char* data);
void uart_init(void);

#endif // UPDATE_GS_H

