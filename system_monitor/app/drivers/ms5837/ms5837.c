// #include <stdio.h>
// #include "ms5837.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// // Private variables
// static uint16_t C[7]; //holds calibration constants and crc
// const static char *TAG = "MS5837";

// // Private functions
// static void bar30_read_raw(i2c_master_dev_handle_t bar30_handle, uint8_t* dataBuffer);
// static int64_t bar30_calculate_temp(uint8_t *raw_temp, int32_t *temp);
// static void bar30_calculate_pressure(int32_t *pressure, int64_t dT, uint8_t *raw_pressure);

// /**
//  * @brief 
//  *
//  * Initialize the bar30 and read the configs from PROM.
//  * 
//  * @details
//  * - the bar30 clocks bytes MSB first, so prom[even] is MSB and prom[odd] is LSB
//  * 
//  * @return returns 0 if successful, 1 if there was an error (e.g. sensor not connected or bad connection)
//  * 
//  * @note 
//  * - the error checking could likely be completed by doing the CRC check on the PROM data. This may even be more robust
//  * @bug Optional known bugs can be listed here.
//  */
// uint8_t bar30_setup(i2c_master_bus_handle_t bus_handle,i2c_master_dev_handle_t bar30_handle){
//     if(bus_handle == NULL || bar30_handle == NULL) {
//         ESP_LOGE(TAG, "Invalid argument: bus_handle and bar30_handle must not be NULL");
//         return 1; // return error code
//     }
//     uint8_t cmd = CMD_MS58XX_RESET; // Send reset command to the sensor
//     uint8_t error = 0;
//     i2c_master_transmit(bar30_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
//     vTaskDelay(pdMS_TO_TICKS(10)); // Delay for sensor reset - this is as per the datasheet
//     // uint8_t read_buffer[2]; // Read calibration coefficients from PROM // DELETE
//     for (uint8_t i = 0; i < 7; i++) {
//         uint8_t data_buffer[2] = {0};
//         cmd = CMD_MS58XX_PROM + (i * 2);
//         i2c_master_transmit_receive(bar30_handle, &cmd, 1, data_buffer, 2, I2C_MASTER_TIMEOUT_MS);
//         C[i] = data_buffer[0] << 8 | data_buffer[1];
        
//         if(!C[i] && !C[i+1]){ // bar30 likes to send back zeros if its not connected properly
//             ESP_LOGE(TAG, "Error reading PROM word %d: received 0x0000, which is invalid. Check sensor connection.", i);
//             error = 1;
//         }
//         else if (i > 1 && i < 7 && C[i] == C[i-1]){ // sometimes it also sends the same thing for all PROM words
//             ESP_LOGE(TAG, "Error reading PROM word %d: received same value as previous word. Check sensor connection.", i);
//             error = 1;
//         }
//         if (error) break;
//     }
//     if(!error){ESP_LOGI(TAG, "MS5837 setup complete. Calibration coefficients read.");}
//     else{ESP_LOGE(TAG, "MS5837 setup incomplete. Errors detected.");}
//     return error;
// }

// /**
//  * @brief 
//  * Read temperature and pressure from Bar30 ADC but do not compute the first/second order corrections.
//  * 
//  * @return N/A
//  * 
//  * @param bar30_handle I2C device handle for the bar30 sensor
//  * @param dataBuffer pointer to a buffer where the raw ADC data will be stored. The first 3 bytes will be pressure data and the next 3 bytes will be temperature data.
//  * 
//  * @note Optional notes or warnings can be included here
//  * @bug Optional known bugs can be listed here.
//  */
// static void bar30_read_raw(i2c_master_dev_handle_t bar30_handle, uint8_t* dataBuffer){
//     // if(bar30_handle == NULL || dataBuffer == NULL) {
//     //     ESP_LOGE(TAG, "Invalid argument: bar30_handle and dataBuffer must not be NULL");
//     //     return;
//     // }
//     // Send command to start pressure conversion (using OSR=256)
//     uint8_t cmd = ADDR_CMD_CONVERT_D1_OSR256;
//     i2c_master_transmit(bar30_handle, &cmd, MS5837_I2C_CMD_SIZE, I2C_MASTER_TIMEOUT_MS);
//     vTaskDelay(pdMS_TO_TICKS(20)); // Delay for conversion - this is as per the datasheet
//     cmd = CMD_MS58XX_READ_ADC;
//     i2c_master_transmit_receive(bar30_handle, &cmd, MS5837_I2C_CMD_SIZE, dataBuffer, MS5837_ADC_READ_SIZE, I2C_MASTER_TIMEOUT_MS);
//     ESP_LOGI(TAG, "Pressure ADC raw data: 0x%02x%02x%02x", dataBuffer[0], dataBuffer[1], dataBuffer[2]); // DELETE
    
//     // Send command to read temperature
//     cmd = ADDR_CMD_CONVERT_D2_OSR256;
//     i2c_master_transmit(bar30_handle, &cmd, MS5837_I2C_CMD_SIZE, I2C_MASTER_TIMEOUT_MS);
//     vTaskDelay(pdMS_TO_TICKS(20)); // max delay as per the datasheet
//     cmd = CMD_MS58XX_READ_ADC;
//     i2c_master_transmit_receive(bar30_handle, &cmd, MS5837_I2C_CMD_SIZE, dataBuffer + 3, MS5837_ADC_READ_SIZE, I2C_MASTER_TIMEOUT_MS);
//     ESP_LOGI(TAG, "Temperature ADC raw data: 0x%02x%02x%02x", dataBuffer[3], dataBuffer[4], dataBuffer[5]); // DELETE
// }

// /**
//  * @brief 
//  *
//  * @return N/A
//  * 
//  * @param bar30_handle i2c device handle
//  * @param dataBuffer pointer to a buffer where the pressure/temperature data will be stored as floats
//  * 
//  * @note Optional notes or warnings can be included here
//  * @bug Optional known bugs can be listed here.
//  */
// void bar30_read(i2c_master_dev_handle_t bar30_handle, uint8_t* dataBuffer){
//     if(bar30_handle == NULL || dataBuffer == NULL) {
//         ESP_LOGE(TAG, "Invalid argument: bar30_handle and dataBuffer must not be NULL");
//         return;
//     }
//     uint8_t raw_buffer[6]; // buffer to hold raw ADC data (3 bytes for pressure, 3 bytes for temperature)
//     int32_t temp = 0;
//     int32_t pressure = 0;
//     int64_t dT = 0;
//     bar30_read_raw(bar30_handle, raw_buffer);
//     dT = bar30_calculate_temp(raw_buffer + 3, &temp);
//     bar30_calculate_pressure(&pressure, dT, raw_buffer);

// }

// /**
//  * @brief 
//  * 
//  * @return N/A
//  * 
//  * @note Optional notes or warnings can be included here
//  * @bug Optional known bugs can be listed here.
//  */
// static int64_t bar30_calculate_temp(uint8_t *raw_temp, int32_t *temp){
// 	int32_t D2_temp = (raw_temp[2] << 16) | (raw_temp[1] << 8) | raw_temp[0];
//     int64_t dT = D2_temp-((uint32_t)C[5] * 256l);
//     *temp = 2000l+(int64_t)dT*C[6]/8388608LL;
//     ESP_LOGI(TAG, "Calculated temperature (before scaling): %d", *temp); // DELETE
//     return dT;
// }

// /**
//  * @brief 
//  * 
//  * @return N/A
//  * 
//  * @note Optional notes or warnings can be included here
//  * @bug Optional known bugs can be listed here.
//  */
// static void bar30_calculate_pressure(int32_t *pressure, int64_t dT, uint8_t *raw_pressure){
//     int64_t OFF = (C[2] << 16) + ((C[4] * dT) >> 7);
//     int64_t SENS = (C[1] << 15) + ((C[3] * dT) >> 8);
//     int64_t raw_pressure_ll = (raw_pressure[2] << 16 | raw_pressure[1] << 8 | raw_pressure[0]);\
//     ESP_LOGI(TAG, "Calculated OFF: %x, SENS: %x, raw_pressure_ll: %x", OFF, SENS, raw_pressure_ll); // DELETE
//     *pressure = (((raw_pressure_ll * SENS) >> 21) - OFF) >> 13;
//     ESP_LOGI(TAG, "Calculated pressure (before scaling): %d", *pressure); // DELETE
// }