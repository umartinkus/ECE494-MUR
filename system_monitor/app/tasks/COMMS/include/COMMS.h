#include "spi_setup.h"

void COMMS(void *arg);

// --- from datapacket.h


// #ifndef DATAPACKET_H
// #define DATAPACKET_H
// #endif // DATAPACKET_H

// #define START_FRAMEH 0X55
// #define START_FRAMEL 0X55

// typedef struct {
//     uint8_t start_frameH;
//     uint8_t start_frameL;
//     uint8_t data_size;
//     uint8_t msg_id;
//     uint8_t data[64];
// } uartPacket_t;

// uartPacket_t make_uart_packet(uint8_t data_size, uint8_t message_id, uint8_t* data);



// -------------------- FROM UPDATE_GS.h -------------------- //
// idf_component_register(SRCS "UPDATE_GS.c"
//                      INCLUDE_DIRS "include"
//                     PRIV_REQUIRES freertos dataPacket esp_driver_uart ms5837 esp_driver_i2c i2c)

// #ifndef UPDATE_GS_H
// #define UPDATE_GS_H

// #define TXD_PIN 32
// #define RXD_PIN 35
// #define RX_BUF_SIZE 1024
// #define BAUD_RATE 115200

// void UPDATE_GS(void *arg);
// int sendData(int size, const char* data);
// void uart_init(void);

// #endif // UPDATE_GS_H