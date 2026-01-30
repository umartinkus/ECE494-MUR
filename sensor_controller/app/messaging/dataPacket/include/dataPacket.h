#ifndef DATAPACKET_H
#define DATAPACKET_H
#include <stdint.h>
#endif // DATAPACKET_H

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
} uartPacket_t;
