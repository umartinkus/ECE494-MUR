#include <stdio.h>
#include "SENSOR.h"
#include "COMMS.h"
#include "sys_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "packet.h"

#define DEFAULT_PRIORITY 5
#define QUEUE_SIZE 64

static QueueHandle_t spi_event_queue;

void app_main(void)
{
    spi_event_queue = xQueueCreate(QUEUE_SIZE, sizeof(packet_t));
    xTaskCreate(COMMS, "comms_task", 4096, (void*)spi_event_queue, DEFAULT_PRIORITY, NULL);

    init_system_state();
    xTaskCreate(SENSOR,"sensor_task", 4096, NULL, DEFAULT_PRIORITY, NULL);
}
