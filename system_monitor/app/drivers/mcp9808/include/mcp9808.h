#ifndef MCP9808_H
#define MCP9808_H

#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MCP9808 and verify communication.
 * * @details Reads the manufacturer ID register to ensure the sensor is connected
 * and responding correctly before proceeding.
 * * @param bus_handle I2C master bus handle
 * @param dev_handle I2C device handle for the MCP9808
 * @return 0 if successful, 1 if there was an error.
 */
uint8_t mcp9808_setup(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t dev_handle);

/**
 * @brief Read the calculated temperature from the MCP9808 in Celsius.
 * * @param dev_handle I2C device handle for the MCP9808
 * @param temperature Pointer to a float where the final temperature will be stored
 */
void mcp9808_read(i2c_master_dev_handle_t dev_handle, float *temperature);

#ifdef __cplusplus
}
#endif

#endif // MCP9808_H
