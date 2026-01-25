#ifndef DATAPACKET_H
#define DATAPACKET_H
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

typedef struct{
    void* pose_buff;
    void* depth_buff;   
} dataBuffers_t;