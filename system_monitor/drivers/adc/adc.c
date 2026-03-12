/*
 * ADC driver currently uses the ESP-IDF oneshot backend.
 * Future work can add a continuous/DMA backend behind the same public API
 * if higher-rate sampling is needed.
 */
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "adc.h"
#include "configuration.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define ADC_DRIVER_MAX_INPUTS 8


/*Helper Functions*/
static error_code_t adc_validate_cfg(const adc_input_cfg_t *cfg);
static adc_unit_state_t *adc_get_unit_state(adc_drv_unit_t unit);
static adc_unit_t adc_map_unit(adc_drv_unit_t unit);
static adc_channel_t adc_map_channel(adc_drv_channel_t channel);
static adc_atten_t adc_map_atten(adc_drv_atten_t attenuation);
static bool adc_cali_supported(void);
static esp_err_t adc_create_calibration(const adc_input_cfg_t *cfg,
                                        adc_cali_handle_t *out_handle);
static void adc_destroy_input_calibration(adc_input_state_t *input_state);
static void adc_cleanup_driver(void);
static adc_input_state_t *adc_find_input_state(const adc_input_cfg_t *input_cfg);


/*Private Structs*/
typedef struct {
    bool initialized;
    adc_oneshot_unit_handle_t handle;
} adc_unit_state_t; // hold status of a one shot unit and its handle

typedef struct {
    bool configured;
    adc_input_cfg_t cfg;
    bool calibration_enabled; // disable if you only want raw vals; it will be less accurate
    adc_cali_handle_t cali_handle;
} adc_input_state_t; 

typedef struct {
    bool initialized;
    adc_unit_state_t adc1;
    adc_unit_state_t adc2;
    adc_input_state_t inputs[ADC_DRIVER_MAX_INPUTS];
    size_t input_count;
} adc_driver_state_t; // status of ADC driver that contains both adcs

/*Public Functions*/
error_code_t adc_driver_init(const adc_input_cfg_t *cfg_table, size_t cfg_count)
{
    size_t i;

    if ((cfg_table == NULL) || (cfg_count == 0U) || (cfg_count > ADC_DRIVER_MAX_INPUTS)) {
        return STATUS_ERROR;
    }

    adc_cleanup_driver();
    memset(&s_adc_driver, 0, sizeof(s_adc_driver));

    for (i = 0; i < cfg_count; i++) {
        const adc_input_cfg_t *cfg = &cfg_table[i];
        adc_unit_state_t *unit_state;
        adc_oneshot_unit_init_cfg_t unit_cfg = {0};
        adc_oneshot_chan_cfg_t chan_cfg = {0};
        esp_err_t err;

        if (adc_validate_cfg(cfg) != STATUS_OK) {
            adc_cleanup_driver();
            return STATUS_ERROR;
        }

        unit_state = adc_get_unit_state(cfg->unit);
        if (unit_state == NULL) {
            adc_cleanup_driver();
            return STATUS_ERROR;
        }

        if (!unit_state->initialized) {
            unit_cfg.unit_id = adc_map_unit(cfg->unit);
            unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

            err = adc_oneshot_new_unit(&unit_cfg, &unit_state->handle);
            if (err != ESP_OK) {
                adc_cleanup_driver();
                return STATUS_ERROR;
            }

            unit_state->initialized = true;
        }

        chan_cfg.atten = adc_map_atten(cfg->attenuation);
        chan_cfg.bitwidth = ADC_BITWIDTH_12;

        err = adc_oneshot_config_channel(unit_state->handle,
                                         adc_map_channel(cfg->channel),
                                         &chan_cfg);
        if (err != ESP_OK) {
            adc_cleanup_driver();
            return STATUS_ERROR;
        }

        s_adc_driver.inputs[i].configured = true;
        s_adc_driver.inputs[i].cfg = *cfg;
        s_adc_driver.inputs[i].calibration_enabled = false;
        s_adc_driver.inputs[i].cali_handle = NULL;

        if (cfg->use_calibration) {
            err = adc_create_calibration(cfg, &s_adc_driver.inputs[i].cali_handle);
            if (err != ESP_OK) {
                adc_cleanup_driver();
                return STATUS_ERROR;
            }

            s_adc_driver.inputs[i].calibration_enabled = true;
        }
    }

    s_adc_driver.input_count = cfg_count;
    s_adc_driver.initialized = true;

    return STATUS_OK;
}

error_code_t adc_driver_read_raw(const adc_input_cfg_t *input_cfg, int *raw_value)
{
    adc_unit_state_t *unit_state;
    esp_err_t err;

    if ((!s_adc_driver.initialized) || (input_cfg == NULL) || (raw_value == NULL)) {
        return STATUS_UNINITIALIZED;
    }

    if (adc_validate_cfg(input_cfg) != STATUS_OK) {
        return STATUS_ERROR;
    }

    unit_state = adc_get_unit_state(input_cfg->unit);
    if ((unit_state == NULL) || (!unit_state->initialized)) {
        return STATUS_ERROR;
    }

    err = adc_oneshot_read(unit_state->handle,
                           adc_map_channel(input_cfg->channel),
                           raw_value);
    if (err != ESP_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

error_code_t adc_driver_read_mv(const adc_input_cfg_t *input_cfg, int *mv_value)
{
    adc_input_state_t *input_state;
    int raw_value = 0;
    esp_err_t err;

    if (!s_adc_driver.initialized) {
        return STATUS_UNINITIALIZED;
    }

    if ((input_cfg == NULL) || (mv_value == NULL)) {
        return STATUS_ERROR;
    }

    input_state = adc_find_input_state(input_cfg);
    if ((input_state == NULL) || (!input_state->calibration_enabled)) {
        return STATUS_ERROR;
    }

    if (adc_driver_read_raw(input_cfg, &raw_value) != STATUS_OK) {
        return STATUS_ERROR;
    }

    err = adc_cali_raw_to_voltage(input_state->cali_handle, raw_value, mv_value);
    if (err != ESP_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

static adc_unit_t adc_map_unit(adc_drv_unit_t unit)
{
    switch (unit) {
    case ADC_DRV_UNIT_1:
        return ADC_UNIT_1;
    case ADC_DRV_UNIT_2:
        return ADC_UNIT_2;
    default:
        return ADC_UNIT_1; /* unreachable after validation */
    }
}

static esp_err_t adc_create_calibration(const adc_input_cfg_t *cfg,
                                        adc_cali_handle_t *out_handle)
{
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = adc_map_unit(cfg->unit),
        .atten = adc_map_atten(cfg->attenuation),
        .bitwidth = ADC_BITWIDTH_12,
    };

    return adc_cali_create_scheme_line_fitting(&cali_cfg, out_handle);
}

static void adc_destroy_input_calibration(adc_input_state_t *input_state)
{
    if ((input_state != NULL) && input_state->calibration_enabled) {
        adc_cali_delete_scheme_line_fitting(input_state->cali_handle);
        input_state->cali_handle = NULL;
        input_state->calibration_enabled = false;
    }
}

static adc_input_state_t *adc_find_input_state(const adc_input_cfg_t *input_cfg)
{
    size_t i;

    for (i = 0; i < s_adc_driver.input_count; i++) {
        adc_input_state_t *state = &s_adc_driver.inputs[i];

        if (!state->configured) {
            continue;
        }

        if ((state->cfg.unit == input_cfg->unit) &&
            (state->cfg.channel == input_cfg->channel) &&
            (state->cfg.attenuation == input_cfg->attenuation) &&
            (state->cfg.use_calibration == input_cfg->use_calibration)) {
            return state;
        }
    }

    return NULL;
}
