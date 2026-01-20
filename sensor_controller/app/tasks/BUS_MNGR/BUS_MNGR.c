#include <stdio.h>
#include "BUS_MNGR.h"
#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "IMU Task";

void BUS_MNGR(void *arg){
    int task_id = (int)arg; // remove
    ESP_LOGI(TAG, "BUS_MNGR Task ID: %d", task_id); // remove
    int core_id = esp_cpu_get_core_id(); // remove
    ESP_LOGI(TAG, "task#%d is running on core#%d", task_id, core_id); // remove
    for(;;){
        initBus();
        vTaskDelay(pdMS_TO_TICKS(4000));
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
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t imu1_handle;
    i2c_master_dev_handle_t imu2_handle;
    i2c_master_init(&bus_handle, &imu1_handle, &imu2_handle); 
    ESP_ERROR_CHECK(mpu9250_register_read(imu1_handle, WHO_AM_I_MPU9250, data, 1));
    if(data[0] == 113)
    {ESP_LOGI(TAG, "IMU1 Active");}
    ESP_ERROR_CHECK(mpu9250_register_read(imu2_handle, WHO_AM_I_MPU9250, data, 1));
    if(data[0] == 113)
    {ESP_LOGI(TAG, "IMU2 Active");}


    // ESP_LOGI(TAG, "I2C initialized successfully"); // TODO: remove

}
