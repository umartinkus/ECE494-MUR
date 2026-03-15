#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "common_types.h"

typedef enum {
    ADC_DRV_UNIT_1 = 0,
    ADC_DRV_UNIT_2,
} adc_drv_unit_t;

typedef enum {
    ADC_DRV_CHANNEL_0 = 0,
    ADC_DRV_CHANNEL_1,
    ADC_DRV_CHANNEL_2,
    ADC_DRV_CHANNEL_3,
    ADC_DRV_CHANNEL_4,
    ADC_DRV_CHANNEL_5,
    ADC_DRV_CHANNEL_6,
    ADC_DRV_CHANNEL_7,
    ADC_DRV_CHANNEL_8,
    ADC_DRV_CHANNEL_9,
} adc_drv_channel_t;

typedef enum {
    ADC_DRV_ATTEN_DB_0 = 0,
    ADC_DRV_ATTEN_DB_2P5,
    ADC_DRV_ATTEN_DB_6,
    ADC_DRV_ATTEN_DB_12,
} adc_drv_atten_t;

typedef struct {
    adc_drv_unit_t unit;
    adc_drv_channel_t channel;
    adc_drv_atten_t attenuation;
    bool use_calibration;
} adc_input_cfg_t;

error_code_t adc_driver_init(const adc_input_cfg_t *cfg_table, size_t cfg_count);
error_code_t adc_driver_read_raw(const adc_input_cfg_t *input_cfg, int *raw_value);
error_code_t adc_driver_read_mv(const adc_input_cfg_t *input_cfg, int *mv_value);
error_code_t adc_driver_deinit(void);
