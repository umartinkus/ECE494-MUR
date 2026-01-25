#include <stdio.h>
#include "THRUST_CTRL.h"
#include "pwm.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define THRUSTER_NEUTRAL_US (1500U)
void THRUST_CTRL(void* params)
{
    thruster_config();
    ESP_ERROR_CHECK(thruster_set_pulse_us(THR_1_CHANNEL, THRUSTER_NEUTRAL_US));
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(thruster_set_pulse_us(THR_1_CHANNEL, 1700U));
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// void app_main(void)
// {
//     // Set the LEDC peripheral configuration
//     example_ledc_init();
//     // Set duty to 50%
//     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, THR_1_CHANNEL, LEDC_DUTY));
//     // Update duty to apply the new value
//     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, THR_1_CHANNEL));
// }
