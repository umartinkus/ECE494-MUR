#include <stdio.h>
#include "sys_common.h"
#include "common_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Shared data structures
static sensor_data_t sensor_data;
static SemaphoreHandle_t sensor_data_mutex; 
static system_status_t system_status;
static SemaphoreHandle_t system_status_mutex; 

static void ensure_system_state_initialized(void)
{
    if (system_status_mutex != NULL && sensor_data_mutex != NULL) {
        return;
    }

    init_system_state();
}

void init_system_state(){
    // Initialize mutexes
    if (system_status_mutex == NULL) {
        system_status_mutex = xSemaphoreCreateMutex();
    }
    if (sensor_data_mutex == NULL) {
        sensor_data_mutex = xSemaphoreCreateMutex();
    }

    if (system_status_mutex != NULL) {
        xSemaphoreTake(system_status_mutex, portMAX_DELAY);
        system_status = (system_status_t){
            .i2c_bus_status = STATUS_UNINITIALIZED,
            .spi_bus_status = STATUS_UNINITIALIZED,
            .pwm_status = STATUS_UNINITIALIZED,
            .imu1_status = STATUS_UNINITIALIZED,
            .imu2_status = STATUS_UNINITIALIZED,
            .ps_status = STATUS_UNINITIALIZED,
        };
        xSemaphoreGive(system_status_mutex);
    }

    if (sensor_data_mutex != NULL) {
        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        sensor_data = (sensor_data_t){0};
        xSemaphoreGive(sensor_data_mutex);
    }
}

//used by SENSOR task
void update_sensor_data(sensor_data_t new_data){
    ensure_system_state_initialized();
    if (sensor_data_mutex == NULL) {
        return;
    }

    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    sensor_data = new_data;
    xSemaphoreGive(sensor_data_mutex);
}

// used by health monitor task
void update_system_status(system_status_t new_status){
    ensure_system_state_initialized();
    if (system_status_mutex == NULL) {
        return;
    }

    xSemaphoreTake(system_status_mutex, portMAX_DELAY);
    system_status = new_status;
    xSemaphoreGive(system_status_mutex);
}

void update_spi_bus_status(error_code_t new_status){
    ensure_system_state_initialized();
    if (system_status_mutex == NULL) {
        return;
    }

    xSemaphoreTake(system_status_mutex, portMAX_DELAY);
    system_status.spi_bus_status = new_status;
    xSemaphoreGive(system_status_mutex);
}

// will be used by health monitor task to update the system status
void get_sensor_data(sensor_data_t* data_out){
    ensure_system_state_initialized();
    if (sensor_data_mutex == NULL) {
        if (data_out != NULL) {
            *data_out = (sensor_data_t){0};
        }
        return;
    }

    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    if(data_out != NULL) *data_out = sensor_data;
    // 2do: some kind of error handling
    xSemaphoreGive(sensor_data_mutex);
}

// will be used by comms task to report system status to the jetson
void get_system_status(system_status_t* status_out){
    ensure_system_state_initialized();
    if (system_status_mutex == NULL) {
        if (status_out != NULL) {
            *status_out = (system_status_t){
                .i2c_bus_status = STATUS_UNINITIALIZED,
                .spi_bus_status = STATUS_UNINITIALIZED,
                .pwm_status = STATUS_UNINITIALIZED,
                .imu1_status = STATUS_UNINITIALIZED,
                .imu2_status = STATUS_UNINITIALIZED,
                .ps_status = STATUS_UNINITIALIZED,
            };
        }
        return;
    }

    xSemaphoreTake(system_status_mutex, portMAX_DELAY);
    if(status_out != NULL) *status_out = system_status; 
    // 2do: some kind of error handling
    xSemaphoreGive(system_status_mutex);
}
