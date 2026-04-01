#include <stdio.h>
#include "SENSOR.h"
#include "i2c1.h"
#include "adc.h"
#include "common_types.h"
#include "configuration.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sys_common.h"

/*Constants*/
// #define DEBUG
#define ADC_COUNT 2
#define LEAK1_ADC_INPUT 0 // maps to index of the array adc_inputs
#define LEAK2_ADC_INPUT 1
#define BATT1_ADC_INPUT 2
#define BATT2_ADC_INPUT 3
#define LEAK_THRESHOLD_MV 450

/*Private variables*/
static sensor_data_t sensor_data;
static i2c_config_t i2c1_config;
#ifdef DEBUG
const static char *TAG = "SENSOR";
#endif

/*Private Functions*/
// static error_code_t check_leak(adc_input_cfg_t *leak_sensor);

void SENSOR(void*){
    // 0. Initialize sensor data variable
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
    if(adc_driver_init(adc_inputs,ADC_COUNT) == STATUS_OK){
        system_status_t sys_update = {STATUS_UNINITIALIZED};
        get_system_status(&sys_update);
        sys_update.leak1_status = STATUS_OK;
        sys_update.leak2_status = STATUS_OK; 
        sys_update.batt1_status = STATUS_OK;    
        sys_update.batt2_status = STATUS_OK;
        update_system_status(sys_update);
    };
    for(;;){
        if(adc_driver_read_mv(&adc_inputs[LEAK1_ADC_INPUT], &sensor_data.leak1) != STATUS_OK){
            // UPDATE SYS STATUS WITH LEAK1 DETECTION ERROR/MALFUNCTION
        }
        if(sensor_data.leak1 > LEAK_THRESHOLD_MV){
            system_status_t sys_update = {STATUS_UNINITIALIZED};
            get_system_status(&sys_update);
            sys_update.leak1_status = STATUS_LEAK_DETECTED;
            update_system_status(sys_update);
        }
        
        if(adc_driver_read_mv(&adc_inputs[LEAK2_ADC_INPUT], &sensor_data.leak2) != STATUS_OK){
            // UPDATE SYS STATUS WITH LEAK2 DETECTION ERROR/MALFUNCTION
        }

        if(sensor_data.leak2 > LEAK_THRESHOLD_MV){
            system_status_t sys_update = {STATUS_UNINITIALIZED};
            get_system_status(&sys_update);
            sys_update.leak2_status = STATUS_LEAK_DETECTED;
            update_system_status(sys_update);
        }

        if(adc_driver_read_mv(&adc_inputs[BATT1_ADC_INPUT], &sensor_data.batt1) != STATUS_OK){
            // UPDATE SYS STATUS WITH BATT1 DETECTION ERROR/MALFUNCTION
            system_status_t sys_update = {STATUS_UNINITIALIZED};
            get_system_status(&sys_update);
            sys_update.batt1_status = STATUS_ERROR;
            update_system_status(sys_update);

        }

        if(adc_driver_read_mv(&adc_inputs[BATT2_ADC_INPUT], &sensor_data.batt2) != STATUS_OK){
            // UPDATE SYS STATUS WITH BATT2 DETECTION ERROR/MALFUNCTION
            system_status_t sys_update = {STATUS_UNINITIALIZED};
            get_system_status(&sys_update);
            sys_update.batt2_status = STATUS_ERROR;
            update_system_status(sys_update);
        }
        #ifdef DEBUG
        ESP_LOGI(TAG,"LEAK1: %D, LEAK2: %D, BATT1: %D, BATT2: %D",sensor_data.leak1, sensor_data.leak2, sensor_data.batt1, sensor_data.batt2);
        #endif
        // 1. 2do read from temp1
        sensor_data.temp1 = 0.0f; 
        // 2. 2do read from temp2
        sensor_data.temp2 = 0.0f; 

        // put everything into global sensor_data variable
        update_sensor_data(sensor_data);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
