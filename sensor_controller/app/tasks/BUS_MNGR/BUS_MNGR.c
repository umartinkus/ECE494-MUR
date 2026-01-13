#include <stdio.h>
#include "BUS_MNGR.h"
#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "create task example";

void BUS_MNGR(void *arg){
    int task_id = (int)arg;
    ESP_LOGI(TAG, "BUS_MNGR Task ID: %d", task_id);
    for(;;){
        int core_id = esp_cpu_get_core_id();
        ESP_LOGI(TAG, "task#%d is running on core#%d", task_id, core_id);
        testBus();
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void testBus()
{
    uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle); 
    ESP_LOGI(TAG, "I2C initialized successfully");
    ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
}
