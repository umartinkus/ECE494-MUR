#pragma once

typedef enum error_code{
    STATUS_OK = 0,
    STATUS_ERROR = 1,
    STATUS_UNINITIALIZED = 2,
    STATUS_UNKNOWN = 99
} error_code_t;

typedef struct{
    float temp1;
    float temp2;
    float leak1;
    float leak2;
    float batt1;
    float batt2;
} sensor_data_t;

typedef struct system_status{
    error_code_t i2c_bus_status;
    error_code_t temp1_status;
    error_code_t temp2_status;
    error_code_t spi_bus_status;
} system_status_t;