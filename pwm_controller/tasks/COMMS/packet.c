#include "common_types.h"
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

esp_err_t transfer_packet(uint8_t size, uint8_t address, const uint8_t* data, QueueHandle_t queue, packet_t* rx_packet_out) {
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

    #ifdef DEBUG
    ESP_LOGI(
        PACKET_TAG,
        "tx hdr size=%u addr=%u crc=%04X",
        packet.data_size,
        packet.device_address,
        packet.crc
    );
    ESP_LOGI(
        PACKET_TAG,
        "tx[00..15] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        s_tx_buf[0], s_tx_buf[1], s_tx_buf[2], s_tx_buf[3],
        s_tx_buf[4], s_tx_buf[5], s_tx_buf[6], s_tx_buf[7],
        s_tx_buf[8], s_tx_buf[9], s_tx_buf[10], s_tx_buf[11],
        s_tx_buf[12], s_tx_buf[13], s_tx_buf[14], s_tx_buf[15]
    );
    ESP_LOGI(
        PACKET_TAG,
        "tx[16..31] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        s_tx_buf[16], s_tx_buf[17], s_tx_buf[18], s_tx_buf[19],
        s_tx_buf[20], s_tx_buf[21], s_tx_buf[22], s_tx_buf[23],
        s_tx_buf[24], s_tx_buf[25], s_tx_buf[26], s_tx_buf[27],
        s_tx_buf[28], s_tx_buf[29], s_tx_buf[30], s_tx_buf[31]
    );
    ESP_LOGI(
        PACKET_TAG,
        "tx[32..47] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        s_tx_buf[32], s_tx_buf[33], s_tx_buf[34], s_tx_buf[35],
        s_tx_buf[36], s_tx_buf[37], s_tx_buf[38], s_tx_buf[39],
        s_tx_buf[40], s_tx_buf[41], s_tx_buf[42], s_tx_buf[43],
        s_tx_buf[44], s_tx_buf[45], s_tx_buf[46], s_tx_buf[47]
    );
    ESP_LOGI(
        PACKET_TAG,
        "tx[48..63] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        s_tx_buf[48], s_tx_buf[49], s_tx_buf[50], s_tx_buf[51],
        s_tx_buf[52], s_tx_buf[53], s_tx_buf[54], s_tx_buf[55],
        s_tx_buf[56], s_tx_buf[57], s_tx_buf[58], s_tx_buf[59],
        s_tx_buf[60], s_tx_buf[61], s_tx_buf[62], s_tx_buf[63]
    );
    #endif
    esp_err_t ret = spi_transaction(s_tx_buf, s_rx_buf, PACKET_SIZE);

    if (ret != ESP_OK) {
        if (ret == ESP_ERR_TIMEOUT) {
            #ifdef DEBUG
            ESP_LOGW(PACKET_TAG, "spi transaction timed out waiting for master");
            #endif
        } else {
            #ifdef DEBUG
            ESP_LOGE(PACKET_TAG, "spi transaction failed: %s", esp_err_to_name(ret));
            #endif
        }
        return ret;
    }


    packet_t rx_packet = {0};
    memcpy(&rx_packet, s_rx_buf, sizeof(rx_packet));

    ESP_LOGI(PACKET_TAG, "crc out: %X", packet.crc);
    ESP_LOGI(PACKET_TAG, "rx sync bytes: %X", rx_packet.crc);

    // check the sync
    // check the crc
    // send to queue
    if (rx_packet.start_frameH == SYNC && rx_packet.start_frameL == SYNC) {
        if (check_crc16(&rx_packet)) {
            if (rx_packet_out != NULL) {
                *rx_packet_out = rx_packet;
            }
            xQueueSendToBack(queue, &rx_packet, pdMS_TO_TICKS(10));
        } else {
            #ifdef DEBUG
            ESP_LOGI(PACKET_TAG, "packet failed crc");
            #endif
            return ESP_ERR_INVALID_CRC;
        }
    } else {
        #ifdef DEBUG
        ESP_LOGI(PACKET_TAG, "packet had no sync bytes");
        #endif
        return ESP_ERR_INVALID_RESPONSE;
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
