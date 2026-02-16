#ifndef DATAPACKET_H
#define DATAPACKET_H
#endif // DATAPACKET_H

#define START_FRAMEH 0X55
#define START_FRAMEL 0X55

typedef struct {
    uint8_t start_frameH;
    uint8_t start_frameL;
    uint8_t data_size;
    uint8_t msg_id;
    uint8_t data[64];
} uartPacket_t;

uartPacket_t make_uart_packet(uint8_t data_size, uint8_t message_id, uint8_t* data);