#pragma once
#include <stdint.h>
// #define DEBUG

#define ERROR_CODE_LIST(X) \
    X(STATUS_UNINITIALIZED, 0) \
    X(STATUS_OK, 1) \
    X(STATUS_ERROR, 2) \
    X(STATUS_LEAK_DETECTED, 3) \
    X(STATUS_CRC_FAILED, 4) \
    X(STATUS_UNKNOWN, 99)

typedef enum  {
    #define X(name, value) name = value,
    ERROR_CODE_LIST(X)
    #undef X
} error_code_t;

typedef struct{
    float temp1;
    float temp2;
    int leak1;
    int leak2;
    int batt1;
    int batt2;
} sensor_data_t;

typedef struct system_status{
    error_code_t i2c_bus_status;
    error_code_t spi_bus_status;
    error_code_t adc_status;
    error_code_t leak1_status;
    error_code_t leak2_status;
    error_code_t batt1_status;
    error_code_t batt2_status;
    error_code_t temp2_status;
    error_code_t temp1_status;
} system_status_t;
