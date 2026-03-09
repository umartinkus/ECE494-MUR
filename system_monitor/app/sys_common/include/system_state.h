#pragma once
void init_system_state();
void update_sensor_data(sensor_data_t new_data);
void update_subsystem_status(subsystem_status_t new_status);
void get_sensor_data(sensor_data_t* data_out);
void get_subsystem_status(subsystem_status_t* status_out);