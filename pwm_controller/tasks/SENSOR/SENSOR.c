#include <stdio.h>
#include "SENSOR.h"
#include "i2c1.h"
#include "adc.h"
#include "common_types.h"
#include "configuration.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sys_common.h"

/*Constants*/
// #define DEBUG

/*Private variables*/
static sensor_data_t sensor_data;
static i2c_config_t i2c1_config;
#ifdef DEBUG
const static char *TAG = "SENSOR";
#endif

/*Private Functions*/
// static error_code_t check_leak(adc_input_cfg_t *leak_sensor);

void SENSOR(void* params){
    // 0. Initialize sensor data variable
    // 1. Create and configure I2C1 with temp sensors
    i2c1_master_init(&i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP1_ADDR, &i2c1_config.temp1_handle, &i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP2_ADDR, &i2c1_config.temp2_handle, &i2c1_config.bus_handle);

    // main sensor polling task
    for(;;){
        // 1. 2do read from temp1
        sensor_data.temp1 = 0.0f; 
        // 2. 2do read from temp2
        sensor_data.temp2 = 0.0f; 

        // put everything into global sensor_data variable
        update_sensor_data(sensor_data);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
