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

#define CORE0 0
#define CORE1 1
#define TASK_PRIO_3 3
#define TASK_PRIO_4 4
#define TASK_PRIO_5 5

// Global Message Buffers
static MessageBufferHandle_t slow_lane_buff;
static MessageBufferHandle_t fast_lane_buff;

//  Event Queues
static QueueHandle_t uart_event_queue;

void app_main(void)
{
<<<<<<< HEAD
    // start the uart port
    init();

    // create the buffers and queues
    pose_msg_buffer = xMessageBufferCreate(1024);
    depth_msg_buffer = xMessageBufferCreate(1024);
    uart_event_queue = xQueueCreate(256, sizeof(uint8_t));
    data_buffers.pose_buff = (void*)pose_msg_buffer;
    data_buffers.depth_buff = (void*)depth_msg_buffer;
    // xTaskCreatePinnedToCore(GET_POSE, "POSE_TASK_C0", 4096, (void*)pose_msg_buffer, TASK_PRIO_5, NULL, CORE0);
    // xTaskCreatePinnedToCore(UPDATE_GS, "UPDATE_GS_TASK_C0", 4096, (void*)&data_buffers, TASK_PRIO_4, NULL, CORE1);
    xTaskCreatePinnedToCore(THRUST_CTRL, "THRUST_CTRL_TASK_C0", 4096, (void*)uart_event_queue, TASK_PRIO_4, NULL, CORE0);
    xTaskCreatePinnedToCore(UART_LISTEN, "UART_LISTEN_TASK_C0", 4096, (void*)uart_event_queue, TASK_PRIO_3, NULL, CORE0);
=======
    slow_lane_buff = xMessageBufferCreate(1024);
    fast_lane_buff = xMessageBufferCreate(2048);
    uart_init();
    xTaskCreatePinnedToCore(GET_POSE, "POSE_TASK_C0", 4096, (void*)slow_lane_buff, TASK_PRIO_5, NULL, CORE0);
    xTaskCreatePinnedToCore(UPDATE_GS, "UPDATE_GS_TASK_C0", 4096, (void*)&fast_lane_buff, TASK_PRIO_4, NULL, CORE1);
    // xTaskCreatePinnedToCore(THRUST_CTRL, "THRUST_CTRL_TASK_C0", 4096, NULL, TASK_PRIO_3, NULL, CORE0);
>>>>>>> refs/remotes/origin/sensor_controller
}
