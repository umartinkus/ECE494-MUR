#pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

typedef struct{
    i2c_master_bus_handle_t i2c1_bus_handle;
    i2c_master_dev_handle_t temp1_handle;
    i2c_master_dev_handle_t temp2_handle;
} sensor_config_t;

void func(void);
