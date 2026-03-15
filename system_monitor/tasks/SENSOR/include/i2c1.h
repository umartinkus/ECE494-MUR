#pragma once

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

// structure that holds bus/device handles. This is only accessed from within the SENSOR task/component
typedef struct{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t temp1_handle;
    i2c_master_dev_handle_t temp2_handle;
} i2c_config_t;

void i2c1_master_init(
  i2c_master_bus_handle_t *bus_handle);

void i2c1_master_add_device(uint8_t dev_addr,
  i2c_master_dev_handle_t *dev_handle,
  i2c_master_bus_handle_t *bus_handle);
