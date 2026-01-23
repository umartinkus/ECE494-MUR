#ifndef DATAPACKET_H
#define DATAPACKET_H
#endif // DATAPACKET_H
typedef struct {
    __uint8_t data_size;
    __uint8_t device_address;
    __uint8_t pose_data[20];
} imuPacket_t;

typedef struct {
    __uint8_t data_size;
    __uint8_t device_address;
    __uint8_t depth_data[6];
} depthPacket_t;