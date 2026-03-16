#pragma once
#include <stdint.h>
#include "mpu9250.h"

#define BAR30_READ_BUFFER_SIZE (sizeof(float) * 2U)

#define ERROR_CODE_LIST(X) \
    X(STATUS_UNINITIALIZED, 0) \
    X(STATUS_OK, 1) \
    X(STATUS_ERROR, 2) \
    X(STATUS_LEAK_DETECTED, 3) \
    X(STATUS_CRC_FAILED, 4) \
    X(STATUS_CONFIG_ERR, 5) \
    X(STATUS_TIMEOUT, 6) \
    X(STATUS_UNKNOWN, 99)

typedef enum {
    #define X(name, value) name = value,
    ERROR_CODE_LIST(X)
    #undef X
} error_code_t;

typedef struct{
    mpu9250_data_t imu1;
    mpu9250_data_t imu2;
    uint8_t bar30_data[BAR30_READ_BUFFER_SIZE];
} sensor_data_t;

typedef struct system_status{
    error_code_t i2c_bus_status;
    error_code_t spi_bus_status;
    error_code_t pwm_status;
    error_code_t imu1_status;
    error_code_t imu2_status;
    error_code_t ps_status;
} system_status_t;
