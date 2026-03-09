#include <stdio.h>
#include "HEALTH_MONITOR.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char* TAG = "HEALTH_MONITOR";
const uint8_t TASK_PERIOD_MS = 20; // 50Hz polling rate 

void subsystem_init_default(subsystem_status_t *status)
{
    status->i2c_bus_status = I2C_NOT_INIT;
    status->temp1_status = I2C_NOT_INIT;
    status->temp2_status = I2C_NOT_INIT;
    status->spi_bus_status = SPI_NOT_INIT;
}

void HEALTH_MONITOR(void *pvParameters)
{
    subsystem_status_t *subsystem_status = (subsystem_status_t *)pvParameters;
    for(;;) 
    {


    }
}
