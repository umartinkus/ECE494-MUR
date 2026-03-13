#pragma once
#include "common_types.h"
void init_system_state();
void update_sensor_data(sensor_data_t new_data);
void update_subsystem_status(system_status_t new_status);
void get_sensor_data(sensor_data_t* data_out);
void get_subsystem_status(system_status_t* status_out);
