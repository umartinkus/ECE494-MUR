#include <stdio.h>
#include "GET_POSE.h"
#include "mpu9250.h"
#include "dataPacket.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "IMU Task";
// I2C bus and device handles
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu1_handle;
static i2c_master_dev_handle_t imu2_handle;

// Private IMU data structures
static mpu9250_data_t imu1_data;
static mpu9250_data_t imu2_data;
static imuPacket_t pose_packet = (imuPacket_t){
    .data_size = sizeof(mpu9250_data_t),
    .device_address = 0x00,
    .pose_data = {0}
}; // only one imu packet bc msg buff send copy

// -------------------- TASK LOOP -------------------- //
void GET_POSE(void *pvParameters){
    initBus();
    configureDevices();
    MessageBufferHandle_t imu_buff = (MessageBufferHandle_t) pvParameters;
    vTaskDelay(pdMS_TO_TICKS(1000)); // give time for other tasks to start
    for(;;){
        mpu9250_get_pose(imu1_handle, &imu1_data);
        makePacket(MPU9250_ADDRESS0, &imu1_data);
        xMessageBufferSend(imu_buff, (void*)&pose_packet, sizeof(pose_packet), pdMS_TO_TICKS(10));
        mpu9250_get_pose(imu2_handle, &imu2_data);
        makePacket(MPU9250_ADDRESS1, &imu2_data);
        xMessageBufferSend(imu_buff, (void*)&pose_packet, sizeof(pose_packet), pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// -------------------- PRIVATE FUNCTIONS -------------------- //

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

/**
 * @brief set defualt configs for the two IMUs
 *
 * 
 * 
 * 
 * @return Description of the value returned by the function, or "void" 
 *         if it does not return a value.
 * 
 * @note Optional notes or warnings can be included here.
 * @bug 
 */
void configureDevices()
{
    mpu9250_set_default_config(imu1_handle);
    mpu9250_set_default_config(imu2_handle);
}

void makePacket(__uint8_t device_address, void* imu_data)
{
    mpu9250_data_t* data = (mpu9250_data_t*)imu_data;
    pose_packet.device_address = device_address;
    memcpy(pose_packet.pose_data, data, sizeof(mpu9250_data_t));
}