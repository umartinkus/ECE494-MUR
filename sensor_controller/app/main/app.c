#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "GET_POSE.h"
#include "UPDATE_GS.h"
#include "THRUST_CTRL.h"
#include "dataPacket.h" // might need to move this somewhere else later

#define CORE0 0
#define CORE1 1
#define TASK_PRIO_3 3
#define TASK_PRIO_4 4
#define TASK_PRIO_5 5

static MessageBufferHandle_t pose_msg_buffer;
static MessageBufferHandle_t depth_msg_buffer;
static dataBuffers_t data_buffers;
void app_main(void)
{
    pose_msg_buffer = xMessageBufferCreate(1024);
    depth_msg_buffer = xMessageBufferCreate(1024);
    data_buffers.pose_buff = (void*)pose_msg_buffer;
    data_buffers.depth_buff = (void*)depth_msg_buffer;
    // xTaskCreatePinnedToCore(GET_POSE, "POSE_TASK_C0", 4096, (void*)pose_msg_buffer, TASK_PRIO_5, NULL, CORE0);
    // xTaskCreatePinnedToCore(UPDATE_GS, "UPDATE_GS_TASK_C0", 4096, (void*)&data_buffers, TASK_PRIO_4, NULL, CORE1);
    xTaskCreatePinnedToCore(THRUST_CTRL, "THRUST_CTRL_TASK_C0", 4096, NULL, TASK_PRIO_3, NULL, CORE0);
}
