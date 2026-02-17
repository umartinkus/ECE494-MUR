#include <stdio.h>
#include <math.h>

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

// helper function declaration
void scale(float *f, float max);
void update_thruster_status(float *f);

const float deadzone = 0.05;
const float upper = 1;

const float max_us = 2000;
const float min_us = 1000;
const float ratio_us = (max_us - min_us) / 2;

const float T_inv[N][N] = {
    {0.00238, 0.50000, -0.00274, -0.01584, 2.96937, -3.59868},
    {-0.00238, 0.50000, 0.00274, 0.01584, -2.96937, 3.59868},
    {-0.35487, 0.00023, 0.35511, -2.34665, -1.94485, 0.00000},
    {0.35176, -0.00024, 0.35511, 2.36740, -1.94485, 0.00000},
    {-0.35223, -0.00023, 0.35200, 2.34665, 1.94485, 0.00000},
    {0.35535, 0.00024, 0.35200, -2.36740, 1.94485, 0.00000}
};


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
    float f[N] = {0};
    uint8_t u[N] = {0};
    uartPacket_t packet = {0};

    // block until there is available data in queue
    for (;;) {
        if (xQueueReceive(uart_events, &packet, portMAX_DELAY)) {
            if (packet.device_address == 0xEE) {
                // this detects an error or something if it gets thrown
                // if we need to change modes, it will go here
                continue;
            } else {
                // get the data from the packet
                memcpy(u, packet.data, (size_t)packet.data_size * sizeof(uint8_t));

                // function to calculate the allocation
                ctrl_allocation(u, f);

                // update all the thrusters with the corresponding values
                update_thruster_status(f);

            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // vecmult(T_inv, u, f);
}

void update_thruster_status(float *f) {
    uint32_t f_rounded;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        f_rounded = (uint32_t)roundf(f[i]);
        
        // update the thruster
        thruster_set_pulse_us(i, f_rounded);
    }
}

// helper function that multiplies a vector by a matrix
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

// helper function to array values between upper and lower
void scale(float *f, float max) {
    float ratio = upper / max;
    for (int i = 0; i < N; i++) {
        f[i] = f[i] * ratio;
    }
}

void scale_us(float *f) {
    for (int i = 0; i < N; i++) {
        f[i] = (f[i] + 1) * ratio_us + min_us;
    }
}

void ctrl_allocation(uint8_t *u, float *f) {
    // map the uint8_t value to something between 0 and 1
    float mapped_u[6] = {0};
    for (int i = 0; i < N; i++) {
        mapped_u[i] = (2.0f / 255.0f) * (float)u[i] - 1.0f;
        
        // apply a deadzone to account for controller stuff
        if (fabs(mapped_u[i]) < deadzone) mapped_u[i] = 0;
    }

    // take the desired "wrench" and find the according thrust forces
    vecmult(T_inv, mapped_u, f);

    // find the max value of the array
    float max = 0;
    for (int i = 0; i < N; i++) {
        if (fabs(f[i]) > max) max = fabs(f[i]);
    }

    // check if the current values are out of range [-1, 1] and scale if so
    if ( max > upper ) {
        scale(f, max);
    }
    
    // scale the values to be in us between max_us and min_us
    scale_us(f);

    for (int i = 0; i < 6; i++) {
        ESP_LOGI("thr value", "f1[%i]: %f", i, f[i]);
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
