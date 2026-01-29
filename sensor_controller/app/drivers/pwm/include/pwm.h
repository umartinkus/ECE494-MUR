#ifndef PWM_H
#define PWM_H
#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY      (200)
#define LEDC_MAX_DUTY       ((1U << LEDC_DUTY_RES) - 1U)
#define LEDC_PERIOD_US      (1000000U / LEDC_FREQUENCY)

extern int thruster_gpio_array[6];

void init_pwm_array(ledc_channel_config_t* pwm_arr, int num_channels);
void init_timer(ledc_timer_config_t* ledc_timer);

uint32_t pwm_us_to_duty(uint32_t pulse_us);
esp_err_t thruster_set_pulse_us(ledc_channel_t channel, uint32_t pulse_us);
#endif // PWM_H
