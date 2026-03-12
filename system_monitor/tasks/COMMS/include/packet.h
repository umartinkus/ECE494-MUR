#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/idf_additions.h"

#define SYNC 0x55
#define MAX_DATA_SIZE 58

typedef struct {
    uint8_t start_frameH;
    uint8_t start_frameL;
    uint8_t data_size;
    uint8_t device_address;
    uint8_t data[58];
    uint16_t crc;
} packet_t;

esp_err_t transfer_packet(uint8_t size, uint8_t address, uint8_t* data, QueueHandle_t queue);
void encode_crc16(packet_t* packet);
int check_crc16(packet_t* packet);
