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

typedef struct {
    uint8_t ls_x;
    uint8_t ls_y;
    uint8_t rs_x;
    uint8_t rs_y;
    uint8_t lt;
    uint8_t rt;
    uint8_t lb;
    uint8_t rb;
} thruster_command_t;

static ledc_channel_config_t pwm_channels[NUM_CHANNELS];
static ledc_timer_config_t ledc_timer;

static float map_axis(uint8_t raw);
static float map_trigger_pair(uint8_t lt, uint8_t rt);
static float map_button_pair(uint8_t negative, uint8_t positive);
static void decode_command(const thruster_command_t *command, float *u);
static void vecmult(const float mat[N][N], const float *vec, float *out);
static void scale(float *f, float max);
static void scale_us(float *f);
static void ctrl_allocation(float *u, float *f);
static void set_thrusters_neutral(void);
static void update_thruster_status(const float *f);

void THRUST_CTRL(void *params)
{
    QueueHandle_t spi_events = (QueueHandle_t)params;
    packet_t packet = {0};
    float u[N] = {0};
    float f[N] = {0};

    init_timer(&ledc_timer);
    init_pwm_array(pwm_channels, NUM_CHANNELS);
    set_thrusters_neutral();
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;) {
        if (xQueueReceive(spi_events, &packet, pdMS_TO_TICKS(THRUST_COMMAND_TIMEOUT_MS)) != pdTRUE) {
            set_thrusters_neutral();
            continue;
        }

        if (packet.device_address != THRUSTER_COMMAND_ADDRESS || packet.data_size < sizeof(thruster_command_t)) {
            continue;
        }

        thruster_command_t command = {0};
        memcpy(&command, packet.data, sizeof(command));
        decode_command(&command, u);
        ctrl_allocation(u, f);
        update_thruster_status(f);

        system_status_t sys_update = {0};
        get_system_status(&sys_update);
        sys_update.spi_bus_status = STATUS_OK;
        update_system_status(sys_update);
    }
}

static float map_axis(uint8_t raw)
{
    float mapped = ((float)raw * (2.0f / 255.0f)) - 1.0f;
    if (fabsf(mapped) < deadzone) {
        return 0.0f;
    }

    return mapped;
}

static float map_trigger_pair(uint8_t lt, uint8_t rt)
{
    float mapped = ((float)rt - (float)lt) / 255.0f;
    if (fabsf(mapped) < deadzone) {
        return 0.0f;
    }

    return mapped;
}

static float map_button_pair(uint8_t negative, uint8_t positive)
{
    float mapped = 0.0f;

    if (positive != 0) {
        mapped += 1.0f;
    }
    if (negative != 0) {
        mapped -= 1.0f;
    }

    return mapped;
}

static void decode_command(const thruster_command_t *command, float *u)
{
    memset(u, 0, sizeof(float) * N);

    u[0] = -map_axis(command->ls_y);
    u[1] = map_axis(command->ls_x);
    u[2] = map_trigger_pair(command->lt, command->rt);
    u[3] = map_button_pair(command->lb, command->rb);
    u[4] = -map_axis(command->rs_y);
    u[5] = map_axis(command->rs_x);
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

static void set_thrusters_neutral(void)
{
    for (int i = 0; i < NUM_CHANNELS; i++) {
        ESP_ERROR_CHECK(thruster_set_pulse_us(i, THRUSTER_NEUTRAL_US));
    }
}

static void update_thruster_status(const float *f)
{
    for (int i = 0; i < NUM_CHANNELS; i++) {
        uint32_t pulse_us = (uint32_t)lroundf(f[i]);
        ESP_ERROR_CHECK(thruster_set_pulse_us(i, pulse_us));
    }
}
