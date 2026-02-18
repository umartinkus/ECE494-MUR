// #pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_NUM 0
#define I2C_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000
#define MS5837_ADC_READ_SIZE 3
#define MS5837_I2C_CMD_SIZE 1

#define CMD_MS58XX_RESET 0x1E
#define CMD_MS58XX_READ_ADC 0x00

/* PROM start address */
#define CMD_MS58XX_PROM 0xA0

/* write to one of these addresses to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR256  0x40
#define ADDR_CMD_CONVERT_D1_OSR512  0x42
#define ADDR_CMD_CONVERT_D1_OSR1024 0x44
#define ADDR_CMD_CONVERT_D1_OSR2048 0x46
#define ADDR_CMD_CONVERT_D1_OSR4096 0x48
#define ADDR_I2C_MS5837 0x76

/* write to one of these addresses to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR256  0x50
#define ADDR_CMD_CONVERT_D2_OSR512  0x52
#define ADDR_CMD_CONVERT_D2_OSR1024 0x54
#define ADDR_CMD_CONVERT_D2_OSR2048 0x56
#define ADDR_CMD_CONVERT_D2_OSR4096 0x58

void i2c_master_init(
  i2c_master_bus_handle_t *bus_handle,
  i2c_master_dev_handle_t *bar30_handle);

uint8_t bar30_setup(
  i2c_master_bus_handle_t bus_handle,
  i2c_master_dev_handle_t dev_handle);

uint8_t bar30_read(
  i2c_master_dev_handle_t dev_handle,
  uint8_t* dataBuffer);

