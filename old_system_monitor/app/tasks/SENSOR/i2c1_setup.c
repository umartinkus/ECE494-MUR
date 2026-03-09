#include <stdio.h>
#include "configuration.h"
#include "i2c1_setup.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
const static char *TAG = "setup";

/**
 * @brief 
 *
 * This function will create a bus handle and a device handle for the dev.
 * 
 * 
 * 
 * @return N/A
 * 
 * @note Optional notes or warnings can be included here
 * @bug Optional known bugs can be listed here.
 */
I2C_ERR_t i2c1_master_init(i2c_master_bus_handle_t *bus_handle){

    if(bus_handle == NULL) {
        ESP_LOGE(TAG, "Invalid argument: bus_handle and dev_handle must not be NULL");
        return I2C_BUS_NULL_PTR;
    }
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C1_MASTER_NUM,
        .sda_io_num = I2C1_MASTER_SDA_IO,
        .scl_io_num = I2C1_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    int retry_count = 0;
    while(i2c_new_master_bus(&bus_config, bus_handle) != ESP_OK && retry_count++ < 5){
        // ESP_LOGI(TAG, "Failed to initialize I2C bus, retrying... (%d/5)", retry_count + 1);
    }
    if(retry_count >= 5) {
        // ESP_LOGE(TAG, "Failed to initialize I2C bus after 5 attempts");
        return I2C_BUS_INIT_FAIL;
    }
    return I2C_OK;
}

/**
 * @brief 
 *
 * Add a device to i2c1 bus
 * 
 * 
 * 
 * @return N/A
 * 
 * @note Optional notes or warnings can be included here
 * @bug Optional known bugs can be listed here.
 */
I2C_ERR_t i2c1_master_add_device(uint8_t dev_addr,
    i2c_master_dev_handle_t *dev_handle,
    i2c_master_bus_handle_t *bus_handle)
{
    if(bus_handle == NULL){
        return I2C_BUS_NULL_PTR;
    }
    if(dev_handle == NULL){
        return I2C_DEV_NULL_PTR;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C1_FREQ_HZ,
    };
    int retry_count = 0;
    while(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle) != ESP_OK && retry_count++ < 5){
        // ESP_LOGI(TAG, "Failed to add device to I2C bus, retrying... (%d/5)", retry_count + 1);
    }
    
    if(retry_count >= 5) {
        // ESP_LOGE(TAG, "Failed to add device to I2C bus after 5 attempts");
        return I2C_DEV_ADD_FAIL;
    }
    return I2C_OK;
}