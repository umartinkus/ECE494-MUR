#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "THRUST_CTRL.h"
#include "esp_err.h"
#include "hal/ledc_types.h"
#include "pwm.h"
#include "state_machine.h"
#include "dataPacket.h"

#define THRUSTER_NEUTRAL_US (1500U)
#define NUM_CHANNELS (6)

static ledc_channel_config_t pwm_channels[NUM_CHANNELS];
static ledc_timer_config_t ledc_timer;

const float deadzone = 0.05;

void THRUST_CTRL(void* params) {
    // initialize the timer and pwm_array
    init_timer(&ledc_timer);
    init_pwm_array(pwm_channels, NUM_CHANNELS);

    // start all of the pwms and neutral then set to 1700us
    for (int i = 0; i < NUM_CHANNELS; i++) {
        ESP_ERROR_CHECK(thruster_set_pulse_us(i, THRUSTER_NEUTRAL_US));
    }
        THRUST_UART_CONS(params);
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

void THRUST_UART_CONS(void* param) {
    QueueHandle_t cmd_queue = (QueueHandle_t)param;

    // create a temp datapacket
    uartPacket_t temp_packet = {0};
    
    // poll forever to get values
    for (;;) {
        if (xQueueReceive(cmd_queue, &thr_cmd, portMAX_DELAY)) {
            update_thruster_status();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
 }
