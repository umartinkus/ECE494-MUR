#include "mpu9250.h"
#include "ak8963.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief 
 *
 * This function will create a bus handle and two device handles. It then opens the I2C bus for IMU1/2
 * 
 * 
 * @return Description of the value returned by the function, or "void" 
 *         if it does not return a value.
 * 
 * @note Optional notes or warnings can be included here.
 * @bug Optional known bugs can be listed here.
 */

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *imu1_handle, 
    i2c_master_dev_handle_t *imu2_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    // configure parameters for imu1
    i2c_device_config_t imu1_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDRESS0,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &imu1_config, imu1_handle));
    
    // configure parameters for imu2
    i2c_device_config_t imu2_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDRESS1,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &imu2_config, imu2_handle));
}

/**
 * @brief 
 *
 * Read from IMU register over I2C
 * 
 * 
 * @return 
 *         
 * 
 * @note Optional notes or warnings can be included here.
 * @bug Optional known bugs can be listed here.
 */
esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
  uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len,
      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief 
 *
 * Write a byte to an IMU register over I2C
 * 
 * 
 * @return 
 *         
 * 
 * @note Optional notes or warnings can be included here.
 * @bug Optional known bugs can be listed here.
 */
 esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
 {
     uint8_t write_buf[2] = {reg_addr, data};
     return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
 }

 /**
 * @brief 
 *
 * Set MPU9250 configuration to default values
 * 
 * 
 * @return 
 *         
 * 
 * @note Optional notes or warnings can be included here.
 * @bug Optional known bugs can be listed here.
 */
void mpu9250_set_default_config(i2c_master_dev_handle_t dev_handle){
    mpu9250_register_write_byte(dev_handle, CONFIG, MPU9250_DEFAULT_LPF_CONFIG); // See page 13 of datasheet; Filter properties BW = 250Hz, Fs = 8kHz
    mpu9250_register_write_byte(dev_handle, SMPLRT_DIV, MPU9250_DEFAULT_SAMPLERATE_DIV); // Set sample rate to 1kHz/4 = 200Hz
    mpu9250_register_write_byte(dev_handle, GYRO_CONFIG, MPU9250_DEFAULT_GYRO_SCALE); // Set gyro full scale to ±2000dps
    mpu9250_register_write_byte(dev_handle, ACCEL_CONFIG, MPU9250_DEFAULT_ACCEL_SCALE); // Set accel full scale to ±8g
    mpu9250_register_write_byte(dev_handle, ACCEL_CONFIG2, MPU9250_DEFAULT_ACCEL_DLPF_CONFIG); // Set accel DLPF to 44Hz
}

 /**
 * @brief 
 *
 * Read accelerometer data from MPU9250
 * 
 * 
 * @return 
 *         
 * 
 * @note Optional notes or warnings can be included here.
 * @bug Optional known bugs can be listed here.
 */
void mpu9250_read_accel(i2c_master_dev_handle_t dev_handle, mpu9250_axis3_i16_t *accel_data){
    uint8_t raw_data[6] = {0};
    mpu9250_register_read(dev_handle, ACCEL_XOUT_H, raw_data, 6); 
    accel_data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    accel_data->y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    accel_data->z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
}

 /**
 * @brief 
 *
 * get pose data from MPU9250
 * 
 * 
 * @return 
 *         
 * 
 * @note Optional notes or warnings can be included here.
 * @bug Optional known bugs can be listed here.
 */

 void mpu9250_get_pose(i2c_master_dev_handle_t dev_handle, mpu9250_data_t *imu_data){
    uint8_t raw_data[14] = {0};
    mpu9250_register_read(dev_handle, ACCEL_XOUT_H, raw_data, 14); 
    imu_data->accel.x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    imu_data->accel.y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    imu_data->accel.z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    imu_data->temp = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    imu_data->gyro.x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    imu_data->gyro.y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    imu_data->gyro.z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    mpu9250_read_mag(dev_handle, &imu_data->mag);
}

 /**
 * @brief 
 *
 * get magnetometer data from MPU9250/AK8963
 * 
 * 
 * @return 
 *         
 * 
 * @note Optional notes or warnings can be included here.
 * @bug Optional known bugs can be listed here.
 */
void mpu9250_read_mag(i2c_master_dev_handle_t dev_handle, mpu9250_axis3_i16_t *mag_data){
    uint8_t raw_data[6] = {0};
    // Enable bypass mode to access magnetometer
    mpu9250_register_write_byte(dev_handle, INT_PIN_CFG, INT_BYPASS_ENABLE);
    // Read magnetometer data
    mpu9250_register_read(dev_handle, AK8963_RA_HXL, raw_data, 6);
    mag_data->x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    mag_data->y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    mag_data->z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
}