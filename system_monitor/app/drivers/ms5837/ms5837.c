#include <stdio.h>
#include "ms5837.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "MS5837";
static uint8_t prom[14]; //holds calibration constants and crc

/**
 * @brief 
 *
 * This function will create a bus handle and a device handle for the bar30 sensor.
 * 
 * 
 * 
 * @return N/A
 * 
 * @note Optional notes or warnings can be included here
 * @bug Optional known bugs can be listed here.
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *bar30_handle){

    if(bus_handle == NULL || bar30_handle == NULL) {
        ESP_LOGE(TAG, "Invalid argument: bus_handle and bar30_handle must not be NULL");
        return;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    int retry_count = 0;
    while(i2c_new_master_bus(&bus_config, bus_handle) != ESP_OK && retry_count++ < 5){
        ESP_LOGI(TAG, "Failed to initialize I2C bus, retrying... (%d/5)", retry_count + 1);
    }

    if(retry_count >= 5) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus after 5 attempts");
        return;
    }
    
    const i2c_device_config_t bar30_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR_I2C_MS5837,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    retry_count = 0;
    while(i2c_master_bus_add_device(*bus_handle, &bar30_config, bar30_handle) != ESP_OK && retry_count++ < 5){
        ESP_LOGI(TAG, "Failed to add MS5837 device to I2C bus, retrying... (%d/5)", retry_count + 1);
    }
    
    if(retry_count >= 5) {
        ESP_LOGE(TAG, "Failed to add MS5837 device to I2C bus after 5 attempts");
        return;
    }
}

/**
 * @brief 
 *
 * Initialize the bar30 and read the configs from PROM.
 * 
 * 
 * 
 * @return N/A
 * 
 * @note Optional notes or warnings can be included here
 * @bug Optional known bugs can be listed here.
 */
uint8_t bar30_setup(i2c_master_bus_handle_t bus_handle,i2c_master_dev_handle_t bar30_handle){
    if(bus_handle == NULL || bar30_handle == NULL) {
        ESP_LOGE(TAG, "Invalid argument: bus_handle and bar30_handle must not be NULL");
        return 1; // return error code
    }
    uint8_t cmd = CMD_MS58XX_RESET; // Send reset command to the sensor
    uint8_t error = 0;
    i2c_master_transmit(bar30_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for sensor reset - this is as per the datasheet
    uint8_t read_buffer[2]; // Read calibration coefficients from PROM
    for (uint8_t i = 0; i < 7; i++) {
        cmd = CMD_MS58XX_PROM + (i * 2);
        i2c_master_transmit_receive(bar30_handle, &cmd, 1, read_buffer, 2, I2C_MASTER_TIMEOUT_MS);
        prom[2*i] = read_buffer[1]; 
        prom[2*i+1] = read_buffer[0]; // storing data in 
        
        if(!read_buffer[0] && !read_buffer[1]){ // bar30 likes to send back zeros if its not connected properly
            ESP_LOGE(TAG, "Error reading PROM word %d: received 0x0000, which is invalid. Check sensor connection.", i);
            error = 1;
        }
        else if (i > 1 && i < 7 && prom[2*i] == prom[2*i-2] && prom[2*i+1] == prom[2*i-1]){ // sometimes it also sends the same thing for all PROM words
            ESP_LOGE(TAG, "Error reading PROM word %d: received same value as previous word. Check sensor connection.", i);
            error = 1;
        }
        if (error) break;
    }
    if(!error){ESP_LOGI(TAG, "MS5837 setup complete. Calibration coefficients read.");}
    else{ESP_LOGE(TAG, "MS5837 setup incomplete. Errors detected.");}
    return error;
}

/**
 * @brief 
 *
 * Read temperature and pressure from Bar30 ADC but do not compute the first/second order corrections.
 * 
 * 
 * 
 * @return N/A
 * 
 * @note Optional notes or warnings can be included here
 * @bug Optional known bugs can be listed here.
 */
void bar30_read(i2c_master_dev_handle_t bar30_handle, uint8_t* dataBuffer){
    if(bar30_handle == NULL || dataBuffer == NULL) {
        ESP_LOGE(TAG, "Invalid argument: bar30_handle and dataBuffer must not be NULL");
        return;
    }
    // Send command to start pressure conversion (using OSR=256)
    uint8_t cmd = ADDR_CMD_CONVERT_D1_OSR256;
    i2c_master_transmit(bar30_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(20)); // Delay for conversion - this is as per the datasheet
    cmd = CMD_MS58XX_READ_ADC;
    i2c_master_transmit_receive(bar30_handle, &cmd, 1, dataBuffer, MS5837_ADC_READ_SIZE, I2C_MASTER_TIMEOUT_MS);
    ESP_LOGI(TAG, "Pressure ADC raw data: 0x%02x%02x%02x", dataBuffer[0], dataBuffer[1], dataBuffer[2]); // 2do: remove
    
    // Send command to read temperature
    cmd = ADDR_CMD_CONVERT_D2_OSR256;
    i2c_master_transmit(bar30_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(20)); // max delay as per the datasheet
    cmd = CMD_MS58XX_READ_ADC;
    i2c_master_transmit_receive(bar30_handle, &cmd, 1, dataBuffer + 3, MS5837_ADC_READ_SIZE, I2C_MASTER_TIMEOUT_MS);
    ESP_LOGI(TAG, "Temperature ADC raw data: 0x%02x%02x%02x", dataBuffer[3], dataBuffer[4], dataBuffer[5]); // 2do: remove
}