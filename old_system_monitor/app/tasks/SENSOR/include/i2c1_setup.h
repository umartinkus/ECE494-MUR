#pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "fault_codes.h"

I2C_ERR_t i2c1_master_init(
  i2c_master_bus_handle_t *bus_handle);

I2C_ERR_t i2c1_master_add_device(uint8_t dev_addr,
  i2c_master_dev_handle_t *dev_handle,
  i2c_master_bus_handle_t *bus_handle);