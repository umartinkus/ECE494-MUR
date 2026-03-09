#include "system_state.h"
#include "../tasks/HEALTH_MONITOR/include/HEALTH_MONITOR.h"
#include "../tasks/SENSOR/include/SENSOR.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Shared data structures
static sensor_data_t sensor_data;
static SemaphoreHandle_t sensor_data_mutex; 
static subsystem_status_t subsystem_status;
static SemaphoreHandle_t subsystem_status_mutex; 

void init_system_state(){
    // Initialize mutexes
    subsystem_status_mutex = xSemaphoreCreateMutex();
    sensor_data_mutex = xSemaphoreCreateMutex();

    //Initialize sensor status
    xSeamphoreTake(subsystem_status_mutex, portMAX_DELAY);
    sensor_status.i2c1_bus_handle = I2C_NOT_INIT;
    sensor_status.temp1_handle = I2C_NOT_INIT;
    sensor_status.temp2_handle = I2C_NOT_INIT;
    xSemaphoreRelease(subsystem_status_mutex);
}

//used by SENSOR task
void update_sensor_data(sensor_data_t new_data){
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    sensor_data = new_data;
    xSemaphoreRelease(sensor_data_mutex);
}

// used by health monitor task
void update_subsystem_status(subsystem_status_t new_status){
    xSemaphoreTake(subsystem_status_mutex, portMAX_DELAY);
    subsystem_status = new_status;
    xSemaphoreRelease(subsystem_status_mutex);
}

// will be used by health monitor task to update the system status
void get_sensor_data(sensor_data_t* data_out){
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    *data_out = (sensor_data == NULL) ? (sensor_data_t){0} : sensor_data; //default return if null
    xSemaphoreRelease(sensor_data_mutex);
}


// will be used by comms task to report system status to the jetson
void get_subsystem_status(subsystem_status_t* status_out){
    xSemaphoreTake(subsystem_status_mutex, portMAX_DELAY);
    *status_out = (subsystem_status == NULL) ? (subsystem_status_t){0} : subsystem_status; //same as above
    xSemaphoreRelease(subsystem_status_mutex);
}