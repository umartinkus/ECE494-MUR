#pragma once
#include <stdint.h>

#define ERROR_CODE_LIST(X) \
    X(STATUS_UNINITIALIZED, 0) \
    X(STATUS_OK, 1) \
    X(STATUS_ERROR, 2) \
    X(STATUS_LEAK_DETECTED, 3) \
    X(STATUS_CRC_FAILED, 4) \
    X(STATUS_UNKNOWN, 99)

typedef enum : uint8_t {
    #define X(name, value) name = value,
    ERROR_CODE_LIST(X)
    #undef X
} error_code_t;

typedef struct{
    
} sensor_data_t;

typedef struct system_status{
    error_code_t i2c_bus_status;
    error_code_t spi_bus_status;
    error_code_t pwm_status;
    error_code_t imu1_status;
    error_code_t imu2_status;
    error_code_t ps_status;
} system_status_t;
