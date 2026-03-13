#include <stdio.h>
#include "SENSOR.h"
#include "i2c1.h"
#include "adc.h"
#include "common_types.h"
#include "configuration.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

/*Constants*/
// #define DEBUG

#define ADC_COUNT 2
#define LEAK1_ADC_INPUT 0 // maps to index of the array adc_inputs
#define LEAK2_ADC_INPUT 1
#define BATT1_ADC_INPUT 2
#define BATT2_ADC_INPUT 3
/*Private variables*/
static sensor_data_t sensor_data;
static i2c_config_t i2c1_config;
const static char *TAG = "SENSOR";

void SENSOR(void*){

    // 1. Create and configure I2C1 with temp sensors
    i2c1_master_init(&i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP1_ADDR, &i2c1_config.temp1_handle, &i2c1_config.bus_handle);
    i2c1_master_add_device(TEMP2_ADDR, &i2c1_config.temp1_handle, &i2c1_config.bus_handle);

    // 2. set up ADCs
    static const adc_input_cfg_t adc_inputs[] = {
    { .unit = ADC_DRV_UNIT_2, .channel = ADC_DRV_CHANNEL_2, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    { .unit = ADC_DRV_UNIT_2, .channel = ADC_DRV_CHANNEL_3, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    { .unit = ADC_DRV_UNIT_1, .channel = ADC_DRV_CHANNEL_6, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    { .unit = ADC_DRV_UNIT_1, .channel = ADC_DRV_CHANNEL_7, .attenuation = ADC_DRV_ATTEN_DB_12, .use_calibration = true },
    };
    adc_driver_init(adc_inputs,ADC_COUNT);
    for(;;){
        if(adc_driver_read_mv(&adc_inputs[LEAK1_ADC_INPUT], &sensor_data.leak1) != STATUS_OK){
            // UPDATE SYS STATUS WITH LEAK1 DETECTION ERROR/MALFUNCTION
        }

        if(adc_driver_read_mv(&adc_inputs[LEAK2_ADC_INPUT], &sensor_data.leak2) != STATUS_OK){
            // UPDATE SYS STATUS WITH LEAK2 DETECTION ERROR/MALFUNCTION
        }

        if(adc_driver_read_mv(&adc_inputs[BATT1_ADC_INPUT], &sensor_data.batt1) != STATUS_OK){
            // UPDATE SYS STATUS WITH BATT1 DETECTION ERROR/MALFUNCTION
        }

        if(adc_driver_read_mv(&adc_inputs[BATT2_ADC_INPUT], &sensor_data.batt1) != STATUS_OK){
            // UPDATE SYS STATUS WITH BATT2 DETECTION ERROR/MALFUNCTION
        }
        #ifdef DEBUG
        ESP_LOGI(TAG,"LEAK1: %D, LEAK2: %D, BATT1: %D, BATT2: %D",sensor_data.leak1, sensor_data.leak2, sensor_data.batt1, sensor_data.batt2);
        #endif
        // 2DO: implement system status updates and sensor data update
        // 1. read from temp1
        // 2. read from temp2
        // 3. read from batt1
        // 4. read from batt2
        // 5. read from leak1
        // 6. read from leak2
        // put everything into sensor_data

        // pull global sensor_data variable
        // *glob_sens_data = sensor_data;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
