#include <math.h>
#include <string.h>
#include "THRUST_CTRL.h"
#include "packet.h"
#include "pwm.h"
#include "configuration.h"
#include "common_types.h"
#include "sys_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#define THRUSTER_NEUTRAL_US (1500U)
#define THRUSTER_MIN_US (1000.0f)
#define THRUSTER_MAX_US (2000.0f)
#define THRUSTER_RANGE_US ((THRUSTER_MAX_US - THRUSTER_MIN_US) / 2.0f)
#define NUM_CHANNELS (6)

static const float deadzone = 0.05f;
static const float upper = 1.0f;
static const float T_inv[N][N] = {
    {0.00238f, 0.50000f, -0.00274f, -0.01584f, 2.96937f, -3.59868f},
    {-0.00238f, 0.50000f, 0.00274f, 0.01584f, -2.96937f, 3.59868f},
    {-0.35487f, 0.00023f, 0.35511f, -2.34665f, -1.94485f, 0.00000f},
    {0.35176f, -0.00024f, 0.35511f, 2.36740f, -1.94485f, 0.00000f},
    {-0.35223f, -0.00023f, 0.35200f, 2.34665f, 1.94485f, 0.00000f},
    {0.35535f, 0.00024f, 0.35200f, -2.36740f, 1.94485f, 0.00000f}
};

static ledc_channel_config_t pwm_channels[NUM_CHANNELS];
static ledc_timer_config_t ledc_timer;

static void vecmult(const float mat[N][N], const float *vec, float *out);
static void scale(float *f, float max);
static void scale_us(float *f);
static void ctrl_allocation(float *u, float *f);
static error_code_t set_thrusters_neutral(void);
static error_code_t update_thruster_status(const float *f);

void THRUST_CTRL(void *params)
{
    QueueHandle_t thrust_cmd_queue = (QueueHandle_t)params;
    packet_t packet = {0};
    float u[N] = {0};
    float f[N] = {0};

    error_code_t tmr_res = init_timer(&ledc_timer);
    error_code_t pwm_res = init_pwm_array(pwm_channels, NUM_CHANNELS);
    
    // get and update system status based off of pwm initializtion
    system_status_t sys_stat = {0};
    get_system_status(&sys_stat);

    if (tmr_res != STATUS_OK || pwm_res != STATUS_OK) {
        sys_stat.pwm_status = STATUS_CONFIG_ERR;
    } else {
        sys_stat.pwm_status = STATUS_OK;
    }

    update_system_status(sys_stat);

    if (sys_stat.pwm_status != STATUS_OK) {
        vTaskDelete(NULL);
    }

    set_thrusters_neutral();
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        get_system_status(&sys_stat);
        if (xQueueReceive(thrust_cmd_queue, &packet, pdMS_TO_TICKS(THRUST_COMMAND_TIMEOUT_MS)) != pdTRUE) {
            set_thrusters_neutral();
            sys_stat.pwm_status = STATUS_TIMEOUT;
            update_system_status(sys_stat);
            continue;
        }

        // get the control values into an array of float
        memset(u, 0, sizeof(u));
        memcpy(u, packet.data, sizeof(u));

        // do the control allocation and update the thruster pwm
        ctrl_allocation(u, f);
        error_code_t update_res = update_thruster_status(f);

        // report any errors
        if (update_res != STATUS_OK) {
            sys_stat.pwm_status = update_res;
            update_system_status(sys_stat);
        }
    }
}

static void vecmult(const float mat[N][N], const float *vec, float *out)
{
    for (int i = 0; i < N; i++) {
        float sum = 0.0f;
        for (int j = 0; j < N; j++) {
            sum += mat[i][j] * vec[j];
        }
        out[i] = sum;
    }
}

static void scale(float *f, float max)
{
    float ratio = upper / max;
    for (int i = 0; i < N; i++) {
        f[i] *= ratio;
    }
}

static void scale_us(float *f)
{
    for (int i = 0; i < N; i++) {
        f[i] = (f[i] + 1.0f) * THRUSTER_RANGE_US + THRUSTER_MIN_US;
    }
}

static void ctrl_allocation(float *u, float *f)
{
    for (int i = 0; i < N; i++) {
        if (fabsf(u[i]) < deadzone) {
            u[i] = 0.0f;
        }
    }

    vecmult(T_inv, u, f);

    float max = 0.0f;
    for (int i = 0; i < N; i++) {
        if (fabsf(f[i]) > max) {
            max = fabsf(f[i]);
        }
    }

    if (max > upper) {
        scale(f, max);
    }

    scale_us(f);
}

static error_code_t set_thrusters_neutral(void) {
    error_code_t pwm_err = STATUS_OK;
    for (int i = 0; i < NUM_CHANNELS; i++) {
         if (thruster_set_pulse_us(i, THRUSTER_NEUTRAL_US) != ESP_OK) {
            pwm_err = STATUS_ERROR;
        }
    }
    return pwm_err;

}

static error_code_t update_thruster_status(const float *f) {
    error_code_t pwm_err = STATUS_OK;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        uint32_t pulse_us = (uint32_t)lroundf(f[i]);
        if (thruster_set_pulse_us(i, pulse_us) != ESP_OK) {
            pwm_err = STATUS_ERROR;
        }
    }
    return pwm_err;
}
