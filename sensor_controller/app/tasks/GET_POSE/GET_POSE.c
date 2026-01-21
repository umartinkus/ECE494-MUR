#include <stdio.h>
#include "GET_POSE.h"
#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "IMU Task";
// I2C bus and device handles
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu1_handle;
static i2c_master_dev_handle_t imu2_handle;

// Global IMU data structures
// static mpu9250_axis3_i16_t imu1_accel_data;
// static mpu9250_axis3_i16_t imu2_accel_data;
static mpu9250_data_t imu1_data;
static mpu9250_data_t imu2_data;

void GET_POSE(void *arg){
    initBus();
    configureDevices();
    vTaskDelay(pdMS_TO_TICKS(1000));
    for(;;){
        // taskENTER_CRITICAL();
        mpu9250_get_pose(imu1_handle, &imu1_data);
        mpu9250_get_pose(imu2_handle, &imu2_data);
        // taskEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/**
 * @brief initialize the i2c bus that hosts imu1 and imu2.
 *
 * This function will create a bus handle, two device handles, and open up and i2c bus. The bus is created
 * successfully if _ function returns true.
 * 
 * 
 * @return Description of the value returned by the function, or "void" 
 *         if it does not return a value.
 * 
 * @note Optional notes or warnings can be included here.
 * @bug When one of the I2C devices fails to initialize, the system enters an infinite boot loop.
 */

void initBus()
{
    uint8_t data[2] = {0, 0};
    i2c_master_init(&bus_handle, &imu1_handle, &imu2_handle); 
    ESP_ERROR_CHECK(mpu9250_register_read(imu1_handle, WHO_AM_I_MPU9250, data, 1));
    if(data[0] == 113){ESP_LOGI(TAG, "IMU1 Active");}
    ESP_ERROR_CHECK(mpu9250_register_read(imu2_handle, WHO_AM_I_MPU9250, data, 1));
    if(data[0] == 113){ESP_LOGI(TAG, "IMU2 Active");}
    ESP_LOGI(TAG, "I2C bus initialized successfully");
}

void configureDevices()
{
    mpu9250_set_default_config(imu1_handle);
    mpu9250_set_default_config(imu2_handle);
}