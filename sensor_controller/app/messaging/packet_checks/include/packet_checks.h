#ifndef PACKET_CHECKS_H
#define PACKET_CHECKS_H
#include "dataPacket.h"
#include <stdbool.h>
#include "crc16_lookup.h"
void encode_crc16(uartPacket_t* packet);
bool check_crc16(uartPacket_t* packet); 
#endif
