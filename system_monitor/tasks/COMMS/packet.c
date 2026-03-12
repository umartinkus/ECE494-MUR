#include "configuration.h"
#include "packet.h"
#include "freertos/idf_additions.h"
#include "spi.h"
#include <string.h>
#include "lookup.h"

esp_err_t transfer_packet(uint8_t size, uint8_t address, uint8_t* data, QueueHandle_t queue) {
    if (size > MAX_DATA_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    packet_t packet = {0};

    // set the individual bytes
    packet.start_frameH = SYNC;
    packet.start_frameL = SYNC;
    packet.data_size = size;
    packet.device_address = address;

    // copy the data
    memcpy(packet.data, data, size);

    // do that crc mf
    encode_crc16(&packet);

    uint8_t* tx_buf = (uint8_t*)&packet;
    uint8_t rx_buf[PACKET_SIZE];

    ESP_ERROR_CHECK(spi_transaction(tx_buf, rx_buf, PACKET_SIZE));
    packet_t rx_packet = *(packet_t*)rx_buf;

    // the next thing to do is check the sync
    // check the crc
    // send to queue xQueueSendToBack
}

void encode_crc16(packet_t* packet) {
    uint16_t crc = 0;
    uint8_t div = 0;

    size_t data_len = packet->data_size;
    if (data_len > sizeof(packet->data)) {
        data_len = sizeof(packet->data);
    }

    // Include header + payload, exclude crc field.
    size_t len = offsetof(packet_t, data) + data_len;

    // Cast the packet's pointer to a byte stream.
    const uint8_t* packet_bytes = (const uint8_t*)packet;
    for (size_t i = 0; i < len; i++) {
        div = (crc >> 8) ^ packet_bytes[i];
        crc = (crc << 8) ^ lookup_bytes[div];
    }

    packet->crc = crc;
}

int check_crc16(packet_t* packet) {
    uint16_t crc = 0;
    uint8_t div = 0;

    size_t data_len = packet->data_size;

    // Include header + payload, exclude crc field.
    size_t len = offsetof(packet_t, data) + data_len;

    // Cast the packet's pointer to a byte stream.
    const uint8_t* packet_bytes = (const uint8_t*)packet;
    for (size_t i = 0; i < len; i++) {
        div = (crc >> 8) ^ packet_bytes[i];
        crc = (crc << 8) ^ lookup_bytes[div];
    }
    
    return crc == packet->crc;
}
