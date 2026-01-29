#pragma once
#include "driver/ledc.h"

// defining the thruster pins
#define THRUSTER_1 (15)
#define THRUSTER_2 (2)
#define THRUSTER_3 (0)
#define THRUSTER_4 (4)
#define THRUSTER_5 (16)
#define THRUSTER_6 (17)

// defining the channels for the thrusters
#define THR_1_CHANNEL LEDC_CHANNEL_0
#define THR_2_CHANNEL LEDC_CHANNEL_1
#define THR_3_CHANNEL LEDC_CHANNEL_2
#define THR_4_CHANNEL LEDC_CHANNEL_3
#define THR_5_CHANNEL LEDC_CHANNEL_4
#define THR_6_CHANNEL LEDC_CHANNEL_5

// setting the pwm settings
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (500)
