#ifndef DATAPACKET_H
#define DATAPACKET_H
#include <stdint.h>

#define START_FRAMEH 0XFF
#define START_FRAMEL 0XFF
typedef struct {
    __uint8_t start_frameH;
    __uint8_t start_frameL;
    __uint8_t data_size;
    __uint8_t device_address;
    __uint8_t pose_data[20];
} imuPacket_t;

typedef struct {
    __uint8_t start_frameH;
    __uint8_t start_frameL;
    __uint8_t data_size;
    __uint8_t device_address;
    __uint8_t depth_data[6];
} depthPacket_t;

typedef struct {
    uint8_t start_frameH;
    uint8_t start_frameL;
    uint8_t data_size;
    uint8_t device_address;
    uint8_t data[64];
    uint16_t crc;
} uartPacket_t;

#endif // DATAPACKET_H
