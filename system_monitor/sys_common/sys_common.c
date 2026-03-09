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

void init_system_state(){
    // Initialize mutexes
    system_status_mutex = xSemaphoreCreateMutex();
    sensor_data_mutex = xSemaphoreCreateMutex();
    //Initialize sensor status
    xSemaphoreTake(system_status_mutex, portMAX_DELAY);
    system_status.i2c_bus_status = STATUS_UNITIALIZED;
    system_status.temp1_status = STATUS_UNITIALIZED;
    system_status.temp2_status = STATUS_UNITIALIZED;
    system_status.spi_bus_status = STATUS_UNITIALIZED;
    // QUESTION: is there a way to do this programatically? So i dont need to repeat?
    xSemaphoreGive(system_status_mutex);
}

//used by SENSOR task
void update_sensor_data(sensor_data_t new_data){
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    sensor_data = new_data;
    xSemaphoreGive(sensor_data_mutex);
}

// used by health monitor task
void update_system_status(system_status_t new_status){
    xSemaphoreTake(system_status_mutex, portMAX_DELAY);
    system_status = new_status;
    xSemaphoreGive(system_status_mutex);
}

// will be used by health monitor task to update the system status
void get_sensor_data(sensor_data_t* data_out){
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    if(data_out != NULL) *data_out = sensor_data;
    // 2do: some kind of error handling
    xSemaphoreGive(sensor_data_mutex);
}

// will be used by comms task to report system status to the jetson
void get_system_status(system_status_t* status_out){
    xSemaphoreTake(system_status_mutex, portMAX_DELAY);
    if(status_out != NULL) *status_out = system_status; 
    // 2do: some kind of error handling
    xSemaphoreGive(system_status_mutex);
}