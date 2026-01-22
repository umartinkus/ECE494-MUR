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
static mpu9250_data_t imu1_data;
static mpu9250_data_t imu2_data;

// sync primitives
// static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
uint64_t get_ms_freertos(){
  return xTaskGetTickCount()/portTICK_PERIOD_MS;
}

void GET_POSE(void *arg){
    initBus();
    configureDevices();
    vTaskDelay(pdMS_TO_TICKS(1000)); // give time for other tasks to start
    for(;;){
        // taskENTER_CRITICAL(&imu_mux); //ensure that one full I2C transaction is done before scheduling another task
        mpu9250_get_pose(imu1_handle, &imu1_data);
        // ESP_LOGI(TAG, "IMU1 Accel => X: %d, Y: %d, Z: %d Gyro => X: %d, Y: %d, Z: %d Mag => X: %d, Y: %d, Z: %d Temp => %d", imu1_data.accel.x, imu1_data.accel.y, imu1_data.accel.z,imu1_data.gyro.x, imu1_data.gyro.y, imu1_data.gyro.z,imu1_data.mag.x, imu1_data.mag.y, imu1_data.mag.z,imu1_data.temp);
        mpu9250_get_pose(imu2_handle, &imu2_data);
        // ESP_LOGI(TAG, "IMU2 Accel => X: %d, Y: %d, Z: %d Gyro => X: %d, Y: %d, Z: %d Mag => X: %d, Y: %d, Z: %d Temp => %d", imu2_data.accel.x, imu2_data.accel.y, imu2_data.accel.z,imu2_data.gyro.x, imu2_data.gyro.y, imu2_data.gyro.z,imu2_data.mag.x, imu2_data.mag.y, imu2_data.mag.z,imu2_data.temp);
            // taskEXIT_CRITICAL(&imu_mux);
        vTaskDelay(pdMS_TO_TICKS(10000));
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