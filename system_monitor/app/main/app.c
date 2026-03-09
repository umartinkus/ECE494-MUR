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
#include "system_state.h"

#define DEFAULT_PRIORITY 5
void app_main(void)
{
    // setup spi, create comms task

    // start health monitor task

    // Get the sensor_config struct containing the i2c bus and dev handles
    // 2DO: utilize the return values of these I2C functions
    // 2DO: move this to sensor task? This code is tied to setup.c/setup.h, so those need to
    // be moved too
    sensor_config_t* sensor_config = get_sensor_config();
    i2c1_master_init(&(sensor_config->i2c1_bus_handle));// init bus 
    i2c1_master_add_device(TEMP1_ADDR,
        &(sensor_config->temp1_handle),
        &(sensor_config->i2c1_bus_handle));
    i2c1_master_add_device(TEMP2_ADDR,
        &(sensor_config->temp2_handle),
        &(sensor_config->i2c1_bus_handle));
    
      // create sensor task 
      xTaskCreate(SENSOR,"sensor_task", 4096, NULL, DEFAULT_PRIORITY, NULL);
}
