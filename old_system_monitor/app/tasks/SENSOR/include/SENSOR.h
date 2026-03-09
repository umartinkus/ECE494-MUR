#pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "i2c1_setup.h"

typedef struct{
    i2c_master_bus_handle_t i2c1_bus_handle;
    i2c_master_dev_handle_t temp1_handle;
    i2c_master_dev_handle_t temp2_handle;
} sensor_config_t;

typedef struct{
    float temp1;
    float temp2;
    float leak1;
    float leak2;
    float batt1;
    float batt2;
} sensor_data_t;

sensor_config_t* get_sensor_config(void);


void SENSOR(void*);