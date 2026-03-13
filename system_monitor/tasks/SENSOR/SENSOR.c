#include <stdio.h>
#include "SENSOR.h"
#include "i2c1.h"
#include "adc.h"
#include "common_types.h"
#include "configuration.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

/*Private variables*/
static sensor_data_t sensor_data;
static i2c_config_t i2c1_config;
const int adc_count = 2;
static int temp1 = 0;
const static char *TAG = "SENSOR";

void SENSOR(void*){

    // create and configure I2C1 with temp sensors
    i2c1_master_init(&i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP1_ADDR, &i2c1_config.temp1_handle, &i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP2_ADDR, &i2c1_config.temp1_handle, &i2c1_config.bus_handle);
    // 1. set up ADCs
    static const adc_input_cfg_t adc_inputs[] = {
    { .unit = ADC_DRV_UNIT_2, .channel = ADC_DRV_CHANNEL_2, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    { .unit = ADC_DRV_UNIT_2, .channel = ADC_DRV_CHANNEL_3, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    { .unit = ADC_DRV_UNIT_1, .channel = ADC_DRV_CHANNEL_6, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    { .unit = ADC_DRV_UNIT_1, .channel = ADC_DRV_CHANNEL_7, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    };
    adc_driver_init(adc_inputs,adc_count);
    // 2. set up leak detectors
    // 3. set up battery voltage monitors
    for(;;){
        // 2do
        adc_driver_read_mv(adc_inputs, &temp1);
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG,"ADC Read %d",temp1);
        // 1. read from temp1
        // 2. read from temp2
        // 3. read from batt1
        // 4. read from batt2
        // 5. read from leak1
        // 6. read from leak2
        // put everything into sensor_data

        // pull global sensor_data variable
        // *glob_sens_data = sensor_data;
    }
}
