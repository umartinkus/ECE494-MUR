#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "GET_POSE.h"

#define CORE0 0
#define TASK_PRIO_3 3
static MessageBufferHandle_t pose_msg_buffer;
static MessageBufferHandle_t depth_msg_buffer;
void app_main(void)
{
    pose_msg_buffer = xMessageBufferCreate(1024);
    xTaskCreatePinnedToCore(GET_POSE, "POSE_TASK_C0", 4096, (void*)pose_msg_buffer, TASK_PRIO_3, NULL, CORE0);
}
