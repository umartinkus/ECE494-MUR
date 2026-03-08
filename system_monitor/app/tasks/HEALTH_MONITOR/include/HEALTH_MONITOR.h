#pragma once
#include "fault_codes.h"

typedef struct subsystem_status{
    I2C_ERR_t i2c_bus_status;
    I2C_ERR_t temp1_status;
    I2C_ERR_t temp2_status;
    SPI_ERR_t spi_bus_status;
} subsystem_status_t;

// typedef enum HEALTH_STATUS{
//     HEALTH_OK = 0,
//     HEALTH_DEGRADED = 1,
//     HEALTH_FAIL = 2
// } HEALTH_STATUS_t;

void subsystem_init_default(subsystem_status_t *status);
void HEALTH_MONITOR(void *pvParameters);