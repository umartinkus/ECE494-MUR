#pragma once
#include "common_types.h"
void init_system_state();
void update_sensor_data(sensor_data_t new_data);
void update_system_status(system_status_t new_status);
void update_spi_bus_status(error_code_t new_status);
void get_sensor_data(sensor_data_t* data_out);
void get_system_status(system_status_t* status_out);
static inline const char *error_code_to_string(error_code_t code)
{
    switch (code) {
#define X(name, value) case name: return #name;
        ERROR_CODE_LIST(X)
#undef X
        default: return "INVALID_ERROR_CODE";
    }
}
