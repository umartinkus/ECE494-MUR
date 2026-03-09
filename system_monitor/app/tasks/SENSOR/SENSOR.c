#include <stdio.h>
#include "SENSOR.h"

static sensor_config_t sensor_task_ctxt; //global var for i2c bus and devices

/**
 * @brief 
 *
 * Get the global variable with i2c handles and bus
 * 
 * 
 * 
 * @return N/A
 * 
 * @note Optional notes or warnings can be included here
 * @bug Optional known bugs can be listed here.
 */
sensor_config_t* get_sensor_config(void){
    return &sensor_task_ctxt;
}


void SENSOR(void*)
{

}

//read from temp1
//read from temp2
//read from leak1
//read from leak2
//read from batt1
//read from batt2
