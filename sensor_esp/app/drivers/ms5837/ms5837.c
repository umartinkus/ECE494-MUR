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

    i2c_master_bus_config_t bus_config = {
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
    
    i2c_device_config_t bar30_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR_I2C_MS5837,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    retry_count = 0;
    while(i2c_master_bus_add_device(*bus_handle, &bar30_config, bar30_handle) != ESP_OK && retry_count++ < 5){
        ESP_LOGI(TAG, "Failed to add MS5837 device to I2C bus, retrying... (%d/5)", retry_count + 1);
    }
}

void bar30_setup(i2c_master_bus_handle_t bus_handle,i2c_master_dev_handle_t bar30_handle){
    
    i2c_master_init(&bus_handle, &bar30_handle);
    // Send reset command to the sensor
    uint8_t cmd = CMD_MS58XX_RESET;
    i2c_master_transmit(bar30_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for sensor reset - this is as per the datasheet

    // Read calibration coefficients from PROM
    uint8_t read_buffer[2];
    for (uint8_t i = 0; i < 7; i++) {
        cmd = CMD_MS58XX_PROM + (i * 2);
        ESP_LOGI(TAG, "Reading PROM word %d from address 0x%02x", i, cmd); // 2do: remove
        i2c_master_transmit_receive(bar30_handle, &cmd, 1, read_buffer, 2, I2C_MASTER_TIMEOUT_MS);
        ESP_LOGI(TAG, "Read Prom word: %d, value: 0x%04x", i, (read_buffer[1] << 8) | read_buffer[0]); // 2do: test this and remove the debug log
        prom[2*i] = read_buffer[1]; // LSB from bar30
        prom[2*i+1] = read_buffer[0]; // MSB from bar30
        ESP_LOGI(TAG, "Stored in Prom[%d]: 0x%02x%02x", 2*i, prom[2*i+1], prom[2*i]); // 2do: remove

        // 2do: ERROR CHECKING
    }
    ESP_LOGI(TAG, "MS5837 setup complete. Calibration coefficients read.");
}

