#include <stdio.h>
#include "UPDATE_GS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataPacket.h"
const static char *TAG = "UPDATE_GS Task";

__uint8_t buffer[22];
// -------------------- TASK LOOP -------------------- //
void UPDATE_GS(void *arg)
{
    dataBuffers_t* data_buffers = (dataBuffers_t*) arg;
    MessageBufferHandle_t pose_msg_buffer = (MessageBufferHandle_t)(data_buffers->pose_buff);
    MessageBufferHandle_t depth_msg_buffer = (MessageBufferHandle_t)(data_buffers->depth_buff);

    for(;;)
    {
        // Placeholder for future implementation
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
        xMessageBufferReceive(pose_msg_buffer, buffer, 22, portMAX_DELAY);
        ESP_LOGI(TAG, "Size of data: %d, address: %x", buffer[0], buffer[1]);
    }
}
