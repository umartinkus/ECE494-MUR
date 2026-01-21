#pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include <stdbool.h>

// ESP32 I2C Parameters
#define I2C_MASTER_SCL_IO           22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// MPU9250 I2C Registers
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
#define INT_BYPASS_ENABLE 0X02

#define MPU9250_ADDRESS0 0x68  // Device address when ADO = 0
#define MPU9250_ADDRESS1 0x69  // Device address when ADO = 1

#define MPU9250_DEFAULT_LPF_CONFIG 0x03  // DLPF_CFG = 3; Fs = 8kHz, BW = 44Hz
#define MPU9250_DEFAULT_SAMPLERATE_DIV 0x04  // Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV); 1kHz/(1+4) = 200Hz
#define MPU9250_DEFAULT_GYRO_SCALE  0x18  // ±2000dps
#define MPU9250_DEFAULT_ACCEL_SCALE  0x10  // ±8g
#define MPU9250_DEFAULT_ACCEL_DLPF_CONFIG 0x03  // A_DLPF_CFG = 3; Fs = 1kHz, BW = 44Hz


typedef struct {
    bool gyro_enabled;
    bool accel_enabled;
    bool temp_enabled;
    uint8_t accel_filter_level;
    uint8_t gyro_temp_filter_level;
} mpu9250_config_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9250_axis3_i16_t;

typedef struct
{
    mpu9250_axis3_i16_t accel;
    mpu9250_axis3_i16_t gyro;
    mpu9250_axis3_i16_t mag;
    int16_t temp;
} mpu9250_data_t;


void i2c_master_init(
  i2c_master_bus_handle_t *bus_handle,
  i2c_master_dev_handle_t *imu1_handle,
  i2c_master_dev_handle_t *imu2_handle);

esp_err_t mpu9250_register_read(
  i2c_master_dev_handle_t dev_handle,
  uint8_t reg_addr, 
  uint8_t *data,
  size_t len);

esp_err_t mpu9250_register_write_byte(
  i2c_master_dev_handle_t dev_handle,
  uint8_t reg_addr,
  uint8_t data);

void mpu9250_set_default_config(
  i2c_master_dev_handle_t dev_handle);


void mpu9250_read_accel(
  i2c_master_dev_handle_t dev_handle,
  mpu9250_axis3_i16_t *accel_data);


void mpu9250_get_pose(
  i2c_master_dev_handle_t dev_handle,
  mpu9250_data_t *imu_data
);

void mpu9250_read_mag(
  i2c_master_dev_handle_t dev_handle,
  mpu9250_axis3_i16_t *mag_data);

// FUTURE
// void mpu9250_read_gyro(
//   i2c_master_dev_handle_t dev_handle,
//   mpu9250_axis3_i16_t *gyro_data);  

//FUTURE
// void mpu9250_read_temp(
//   i2c_master_dev_handle_t dev_handle,
//   int16_t *temp_data);

