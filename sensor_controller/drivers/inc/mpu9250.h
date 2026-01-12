#pragma once
#include "driver/i2c_types.h"
#include <stdbool.h>

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR1         0x68        /*!< Address of the MPU9250 sensor */
#define MPU9250_SENSOR_ADDR2         0x69        /*!< Address of the MPU9250 sensor with AD0 high*/
#define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define MPU9250_RESET_BIT           7

typedef struct {
  bool accel_enabled;
  bool gyro_enabled;
  bool temp_enabled;
  // Which level of filtering to use (0-8); higher numbers are less noisy
  // but have reduced sample rate and increased delay.
  int accel_filter_level;
  // Which level of filtering to use (0-9); higher numbers are less noisy
  // but have reduced sample rate and increased delay.
  int gyro_temp_filter_level;
} mpu9250_config_t;

struct mpu9250_vector3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

typedef struct {
  mpu9250_config_t config;
  struct mpu9250_vector3 accel;
  struct mpu9250_vector3 gyro;
  struct mpu9250_vector3 mag;
  float temp;
  i2c_master_dev_handle_t _handle;
} mpu9250_t;

int mpu9250_begin(mpu9250_t *mpu, const mpu9250_config_t config, int address,
                  i2c_master_bus_handle_t i2c_bus_handle);
int mpu9250_update(mpu9250_t *mpu);