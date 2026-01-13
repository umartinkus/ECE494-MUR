#pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include <stdbool.h>

#define I2C_MASTER_SCL_IO           22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define MPU9250_RESET_BIT           7

esp_err_t mpu9250_register_read(
  i2c_master_dev_handle_t dev_handle,
  uint8_t reg_addr, 
  uint8_t *data,
  size_t len);

void i2c_master_init(
  i2c_master_bus_handle_t *bus_handle,
  i2c_master_dev_handle_t *dev_handle);
