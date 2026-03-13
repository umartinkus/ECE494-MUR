#include "configuration.h"
#include "packet.h"
#include "freertos/idf_additions.h"
#include "spi.h"
#include <string.h>
#include "lookup.h"
#include "esp_log.h"
#include "esp_attr.h"

#define PACKET_TAG "transfer_packet"

static DMA_ATTR WORD_ALIGNED_ATTR uint8_t s_tx_buf[PACKET_SIZE];
static DMA_ATTR WORD_ALIGNED_ATTR uint8_t s_rx_buf[PACKET_SIZE];

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

    memcpy(s_tx_buf, &packet, sizeof(packet));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));


    ESP_ERROR_CHECK(spi_transaction(s_tx_buf, s_rx_buf, PACKET_SIZE));
    packet_t rx_packet = {0};
    memcpy(&rx_packet, s_rx_buf, sizeof(rx_packet));
    ESP_LOGI(PACKET_TAG, "rx sync bytes: %X", rx_packet.crc);

    // check the sync
    // check the crc
    // send to queue
    if (rx_packet.start_frameH == SYNC && rx_packet.start_frameL == SYNC) {
        if (check_crc16(&rx_packet)) {
            xQueueSendToBack(queue, &rx_packet, pdMS_TO_TICKS(10));
        } else {
            ESP_LOGI(PACKET_TAG, "packet failed crc");
        }
    } else {
        ESP_LOGI(PACKET_TAG, "packet had no sync bytes");
    }

    return ESP_OK;
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
    if (data_len > sizeof(packet->data)) {
        return 0;
    }

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
