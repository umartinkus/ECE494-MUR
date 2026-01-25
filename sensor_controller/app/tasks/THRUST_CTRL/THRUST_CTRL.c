#include <stdio.h>
#include "THRUST_CTRL.h"
#include "hal/ledc_types.h"
#include "pwm.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define THRUSTER_NEUTRAL_US (1500U)
#define NUM_CHANNELS (6)

static ledc_channel_config_t pwm_channels[NUM_CHANNELS];
static ledc_timer_config_t ledc_timer;

void THRUST_CTRL(void* params) {
  // initialize the timer and pwm_array
  init_timer(&ledc_timer);
  init_pwm_array(pwm_channels, NUM_CHANNELS);
  
  // start all of the pwms and neutral then set to 1700us
  for (int i = 0; i < NUM_CHANNELS; i++) {
    ESP_ERROR_CHECK(thruster_set_pulse_us(i, THRUSTER_NEUTRAL_US));
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(thruster_set_pulse_us(i, 1700U));
  }

  // keep task running as not to reset the pwm signals
  for (;;) {
      vTaskDelay(pdMS_TO_TICKS(3000));
  }
}
