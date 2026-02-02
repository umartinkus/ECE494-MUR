#include <stddef.h>
#include "packet_checks.h"
#include "dataPacket.h"

void encode_crc16(uartPacket_t* packet) {
    uint16_t crc = 0;
    uint8_t div = 0;

    size_t data_len = packet->data_size;
    if (data_len > sizeof(packet->data)) {
        data_len = sizeof(packet->data);
    }

    // Include header + payload, exclude crc field.
    size_t len = offsetof(uartPacket_t, data) + data_len;

    // Cast the packet's pointer to a byte stream.
    const uint8_t* packet_bytes = (const uint8_t*)packet;
    for (size_t i = 0; i < len; i++) {
        div = (crc >> 8) ^ packet_bytes[i];
        crc = (crc << 8) ^ lookup_bytes[div];
    }

    packet->crc = crc;
}

bool check_crc16(uartPacket_t* packet) {
    uint16_t crc = 0;
    uint8_t div = 0;

    size_t data_len = packet->data_size;

    // Include header + payload, exclude crc field.
    size_t len = offsetof(uartPacket_t, data) + data_len;

    // Cast the packet's pointer to a byte stream.
    const uint8_t* packet_bytes = (const uint8_t*)packet;
    for (size_t i = 0; i < len; i++) {
        div = (crc >> 8) ^ packet_bytes[i];
        crc = (crc << 8) ^ lookup_bytes[div];
    }
    
    return crc == packet->crc;
}
