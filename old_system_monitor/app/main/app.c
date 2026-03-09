#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "SENSOR.h"
#include "HEALTH_MONITOR.h"
#include "COMMS.h"
#define DEFAULT_PRIORITY 5
void app_main(void)
{
    // setup spi, create comms task
    xTaskCreate(COMMS, "comms_task", 4096, NULL, DEFAULT_PRIORITY, NULL);

    // start health monitor task
    xTaskCreate(HEALTH_MONITOR, "health_monitor_task", 4096, NULL, DEFAULT_PRIORITY, NULL);
    
    // create sensor task 
    xTaskCreate(SENSOR,"sensor_task", 4096, NULL, DEFAULT_PRIORITY, NULL);
}
