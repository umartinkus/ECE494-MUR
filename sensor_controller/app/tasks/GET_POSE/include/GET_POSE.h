#ifndef GET_POSE_H
#define GET_POSE_H
void GET_POSE(void *arg);
void initBus();
void configureDevices();
void makePacket(__uint8_t device_address, void* imu_data);
#endif // GET_POSE_H