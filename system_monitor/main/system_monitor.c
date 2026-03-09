#include <stdio.h>
#include "SENSOR.h"
#include "COMMS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#define DEFAULT_PRIORITY 5

void app_main(void)
{
    xTaskCreate(SENSOR,"sensor_task", 4096, NULL, DEFAULT_PRIORITY, NULL);
    xTaskCreate(COMMS, "comms_task", 4096, NULL, DEFAULT_PRIORITY, NULL);
}
