#include <stdio.h>
#include "dataPacket.h" 
#include "esp_log.h"

uartPacket_t make_uart_packet(uint8_t data_size, uint8_t message_id, uint8_t* data)
{
    if(data_size > 64) {
        ESP_LOGW("DATA_PACKET", "Data size exceeds 64 bytes. Limiting to 64 bytes.");
        return (uartPacket_t){0}; // Return an empty packet or handle as needed
    }
    if(data == NULL) {
        ESP_LOGE("DATA_PACKET", "Data pointer is NULL. Cannot create packet.");
        // Handle this case as needed, e.g., return an empty packet or a packet with an error code
        return (uartPacket_t){0}; // Return an empty packet
    }
    uartPacket_t packet;
    packet.start_frameH = START_FRAMEH;
    packet.start_frameL = START_FRAMEL;
    packet.data_size = data_size;
    packet.msg_id = message_id;
    for(int i = 0; i < data_size && i < 64; i++) {
        packet.data[i] = data[i];
    }
    return packet;
}
