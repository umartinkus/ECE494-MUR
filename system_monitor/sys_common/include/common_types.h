#pragma once

typedef enum error_code{
    STATUS_UNINITIALIZED = 0,
    STATUS_OK = 1,
    STATUS_ERROR = 2,
    STATUS_LEAK_DETECTED = 3,
    STATUS_UNKNOWN = 99
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