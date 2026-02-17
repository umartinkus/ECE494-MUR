#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "THRUST_CTRL.h"
#include "esp_log.h"
#include "esp_err.h"
#include "hal/ledc_types.h"
#include "portmacro.h"
#include "pwm.h"
#include "state_machine.h"
#include "dataPacket.h"

#define THRUSTER_NEUTRAL_US (1500U)
#define NUM_CHANNELS (6)

static ledc_channel_config_t pwm_channels[NUM_CHANNELS];
static ledc_timer_config_t ledc_timer;

const float deadzone = 0.05;

const float T_inv[N][N] = {
    {0.00238, 0.50000, -0.00274, -0.01584, 2.96937, -3.59868},
    {-0.00238, 0.50000, 0.00274, 0.01584, -2.96937, 3.59868},
    {-0.35487, 0.00023, 0.35511, -2.34665, -1.94485, 0.00000},
    {0.35176, -0.00024, 0.35511, 2.36740, -1.94485, 0.00000},
    {-0.35223, -0.00023, 0.35200, 2.34665, 1.94485, 0.00000},
    {0.35535, 0.00024, 0.35200, -2.36740, 1.94485, 0.00000}
};

void vecmult(const float mat[N][N], float *vec, float *out) {
    float sum;
    for (int i = 0; i < N; i++) {
        sum = 0; 
        for (int j = 0; j < N; j++) {
            sum += mat[i][j] * vec[j];
        }
        // put sum into out matrix
        out[i] = sum;
    }
}

void THRUST_CTRL(void* params) {
    // convert params
    QueueHandle_t uart_events = (QueueHandle_t)params;

    // initialize the timer and pwm_array
    init_timer(&ledc_timer);
    init_pwm_array(pwm_channels, NUM_CHANNELS);

    // start all of the pwms and neutral then set to 1700us
    for (int i = 0; i < NUM_CHANNELS; i++) {
        ESP_ERROR_CHECK(thruster_set_pulse_us(i, THRUSTER_NEUTRAL_US));
    }

    // wait for a few seconds to unlock thrusters
    vTaskDelay(pdMS_TO_TICKS(1000));

    // temp variables
    float f[6] = {0};
    uartPacket_t packet = {0};

    // block until there is available data in queue
    for (;;) {
        if (xQueueReceive(uart_events, &packet, portMAX_DELAY)) {
            if (packet.device_address == 0xEE) {
                // this detects an error or something if it gets thrown
                // if we need to change modes, it will go here
                continue;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }


    float u[6] = {0, 1, 0, 0, 0, 0};
    vecmult(T_inv, u, f);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void update_thruster_status(bool* thruster_status) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (thruster_status[i]) {
            ESP_ERROR_CHECK(thruster_set_pulse_us(i, 1750U));
        } else {
            ESP_ERROR_CHECK(thruster_set_pulse_us(i, 1500U));
        }
    }
}

// void THRUST_UART_CONS(void* param) {
//     QueueHandle_t cmd_queue = (QueueHandle_t)param;
//
//     // create a temp datapacket
//     uartPacket_t temp_packet = {0};
//
//     // poll forever to get values
//     for (;;) {
//         if (xQueueReceive(cmd_queue, &thr_cmd, portMAX_DELAY)) {
//             update_thruster_status();
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
//  }
