#include <stdio.h>
#include "SENSOR.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"

static sensor_config_t sensor_task_ctxt; //global var for i2c bus and devices

// /**
//  * @brief 
//  *
//  * Get the global variable with i2c handles and bus
//  * 
//  * 
//  * 
//  * @return N/A
//  * 
//  * @note Optional notes or warnings can be included here
//  * @bug Optional known bugs can be listed here.
//  */
// sensor_config_t* get_sensor_config(void){
//     return &sensor_task_ctxt;
// }


void SENSOR(void*)
{
    // Get the sensor_config struct containing the i2c bus and dev handles
    // 2DO: utilize the return values of these I2C functions
    i2c1_master_init(&(sensor_config->i2c1_bus_handle));// init bus 
    i2c1_master_add_device(TEMP1_ADDR,
        &(sensor_config->temp1_handle),
        &(sensor_config->i2c1_bus_handle));
    i2c1_master_add_device(TEMP2_ADDR,
        &(sensor_config->temp2_handle),
        &(sensor_config->i2c1_bus_handle));
    for(;;){

    }
}

//read from temp1
//read from temp2
//read from leak1
//read from leak2
//read from batt1
//read from batt2
