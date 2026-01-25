#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define THRUSTER_1          (15)
#define THRUSTER_2          (2)
#define THRUSTER_3          (0)
#define THRUSTER_4          (4)
#define THRUSTER_5          (16)
#define THRUSTER_6          (17)

#define THR_1_CHANNEL       LEDC_CHANNEL_0

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY      (200)
#define LEDC_MAX_DUTY       ((1U << LEDC_DUTY_RES) - 1U)
#define LEDC_PERIOD_US      (1000000U / LEDC_FREQUENCY)

void thruster_config();
uint32_t pwm_us_to_duty(uint32_t pulse_us);
esp_err_t thruster_set_pulse_us(ledc_channel_t channel, uint32_t pulse_us);
