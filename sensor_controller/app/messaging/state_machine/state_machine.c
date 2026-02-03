#include "state_machine.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "include/state_machine.h"
#include "dataPacket.h"

static const char* TAG = "state_machine";

float thr_inv[6][6] = {
    {  0.000545f,  0.5f,       -0.000003f, -0.000022f, -0.000023f, -0.003693f },
    { -0.000545f,  0.5f,        0.000003f,  0.000022f,  0.000023f,  0.003693f },
    { -0.359017f,  0.0f,        0.349225f,  2.318034f,  2.383222f,  0.0f      },
    {  0.348197f,  0.0f,        0.357989f,  2.318034f, -2.383222f,  0.0f      },
    { -0.348197f,  0.0f,        0.357989f, -2.318034f, -2.383222f,  0.0f      },
    {  0.359017f,  0.0f,        0.349225f, -2.318034f,  2.383222f,  0.0f      }
};

static bool sync_recieved = false;
uartPacket_t packet_buf = {0};
static size_t position = 0;
extern State state;

void sync_state(uint8_t event, QueueHandle_t queue) {
    (void)queue; // this is just to silence the warning
    ESP_LOGI(TAG, "yo it worked: %d", event);

    if (sync_recieved && event == START_FRAMEL) {
        state = size_state;

    } else if (event == START_FRAMEH) {
        sync_recieved = true;
    }
}

void size_state(uint8_t event, QueueHandle_t queue) {
    (void)queue; // this is just to silence the warning

    packet_buf.data_size = event;
    state = addr_state;
}

void addr_state(uint8_t event, QueueHandle_t queue) {
    (void)queue; // this is just to silence the warning

    packet_buf.device_address = event;
    state = data_state;
}

void data_state(uint8_t event, QueueHandle_t queue) {
    if (position < packet_buf.data_size) {
        packet_buf.data[position++] = event;
    } else {
        xQueueSendToBack(queue, &packet_buf, pdMS_TO_TICKS(10));
        state = sync_state;
        sync_recieved = false;
        position = 0;
        ESP_LOGI(TAG, "yo it worked: %d", uxQueueMessagesWaiting(queue));
    }
}
