#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "setup.h"
#include "configuration.h"
#include "SENSOR.h"
#include "HEALTH_MONITOR.h"

// Task contexts
sensor_config_t sensor_task_ctxt; 
subsystem_status_t health_monitor_ctxt; 

void app_main(void)
{
    subsystem_init_default(&health_monitor_ctxt); // setting subsystem status to default vals (health monitor task)
    // setup spi

    // setup i2c
    health_monitor_ctxt.i2c_bus_status = i2c1_master_init(&sensor_task_ctxt.i2c1_bus_handle);
    health_monitor_ctxt.temp1_status = i2c1_master_add_device(TEMP1_ADDR, &sensor_task_ctxt.temp1_handle, &sensor_task_ctxt.i2c1_bus_handle);
    health_monitor_ctxt.temp2_status = i2c1_master_add_device(TEMP2_ADDR, &sensor_task_ctxt.temp2_handle, &sensor_task_ctxt.i2c1_bus_handle);

    // create tasks

    // create buffers


}
