#include <stdio.h>
#include "BUS_MNGR.h"
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
static mpu9250_axis3_i16_t imu1_accel_data;
static mpu9250_axis3_i16_t imu2_accel_data;

void BUS_MNGR(void *arg){
    // int task_id = (int)arg; // remove
    // ESP_LOGI(TAG, "BUS_MNGR Task ID: %d", task_id); // remove
    // int core_id = esp_cpu_get_core_id(); // remove
    // ESP_LOGI(TAG, "task#%d is running on core#%d", task_id, core_id); // remove
    initBus();
    configureDevices();
    vTaskDelay(pdMS_TO_TICKS(1500));
    for(;;){
        mpu9250_read_accel(imu1_handle, &imu1_accel_data);
        ESP_LOGI(TAG, "IMU1 Accel => X: %d, Y: %d, Z: %d", imu1_accel_data.x, imu1_accel_data.y, imu1_accel_data.z);
        mpu9250_read_accel(imu2_handle, &imu2_accel_data);
        ESP_LOGI(TAG, "IMU2 Accel => X: %d, Y: %d, Z: %d", imu2_accel_data.x, imu2_accel_data.y, imu2_accel_data.z);
        vTaskDelay(pdMS_TO_TICKS(500));
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