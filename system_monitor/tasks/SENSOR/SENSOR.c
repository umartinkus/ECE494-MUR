#include <stdio.h>
#include "SENSOR.h"
#include "i2c1.h"
#include "common_types.h"
#include "configuration.h"

static sensor_data_t sensor_data;
static i2c_config_t i2c1_config;

void SENSOR(void*){

    // create and configure I2C1 with temp sensors
    i2c1_master_init(&i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP1_ADDR, &i2c1_config.temp1_handle, &i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP2_ADDR, &i2c1_config.temp1_handle, &i2c1_config.bus_handle);
    // 2do
    // 1. set up ADCs
    // 2. set up leak detectors
    // 3. set up battery voltage monitors
    for(;;){
        // 2do
        // 1. read from temp1
        // 2. read from temp2
        // 3. read from batt1
        // 4. read from batt2
        // 5. read from leak1
        // 6. read from leak2
        // put everything into sensor_data

        // pull global sensor_data variable
        // *glob_sens_data = sensor_data;
    }
}
