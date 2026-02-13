#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"

// tasks
#include "GET_POSE.h"
#include "UPDATE_GS.h"
#include "THRUST_CTRL.h"
#include "UART_LISTEN.h"

// other needed includes
#include "dataPacket.h" // might need to move this somewhere else later
#include "packet_checks.h"

#define CORE0 0
#define CORE1 1
#define TASK_PRIO_3 3
#define TASK_PRIO_4 4
#define TASK_PRIO_5 5
static const char* TAG = "state_machine";

// Global Message Buffers
static MessageBufferHandle_t fast_lane_buff;

//  Event Queues
static QueueHandle_t uart_event_queue;
static QueueHandle_t parsed_data_queue;


void app_main(void)
{
    fast_lane_buff = xMessageBufferCreate(2048);

    // create queues
    uart_event_queue = xQueueCreate(512, sizeof(uint8_t));
    parsed_data_queue = xQueueCreate(128, sizeof(uartPacket_t));

    // instantiate struct for queues
    struct UartVariables uart_pv_params;  // this struct exists in the UART_LISTEN.h
    uart_pv_params.uart_queue = &uart_event_queue;
    uart_pv_params.parsed_queue = &parsed_data_queue;
    uart_pv_params.test = 67;

    ESP_LOGI(TAG, "I hope this says 67: %p", &uart_event_queue);

    uart_init();
    // xTaskCreatePinnedToCore(GET_POSE, "POSE_TASK_C0", 4096, (void*)fast_lane_buff, TASK_PRIO_5, NULL, CORE0);
    // xTaskCreatePinnedToCore(THRUST_CTRL, "THRUST_CTRL_TASK_C0", 4096, (void*)uart_event_queue, TASK_PRIO_3, NULL, CORE0);
    xTaskCreatePinnedToCore(UART_LISTEN, "UART_LISTEN_TASK_C0", 4096, (void*)&uart_pv_params, TASK_PRIO_4, NULL, CORE0);
}
