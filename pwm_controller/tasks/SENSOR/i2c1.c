#include "i2c1.h"
#include "sys_common.h"
#include "configuration.h"

/**
 * @brief 
 *
 * Create an i2c bus handle using the I2C1 interface 
 * 
 * 
 * 
 * @return N/A
 * @param bus_handle pointer to the i2c bus handle
 * @note if the bus initialization is unsuccessful, the system state is updated with a failed I2C bus
 * @bug Optional known bugs can be listed here.
 */
void i2c1_master_init(i2c_master_bus_handle_t *bus_handle){
    if(bus_handle == NULL) {
        // 1. pull the system state,
        // 2. update the i2c bus member in sys state
        // 3. push the updated sys state to the global
        system_status_t sys_update = {0};
        get_system_status(&sys_update);
        sys_update.i2c_bus_status = STATUS_ERROR;
        update_system_status(sys_update);
        return;
    }
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C1_MASTER_NUM,
        .sda_io_num = I2C1_MASTER_SDA_IO,
        .scl_io_num = I2C1_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    int retry_count = 0;
    while(i2c_new_master_bus(&bus_config, bus_handle) != ESP_OK && retry_count++ < 5){
    }
    if(retry_count >= 5) {
        // 1. pull the system state,
        // 2. update the i2c bus member in sys state
        // 3. push the updated sys state to the global state
        system_status_t sys_update = {0};
        get_system_status(&sys_update);
        sys_update.i2c_bus_status = STATUS_ERROR;
        update_system_status(sys_update);
    }
}

/**
 * @brief 
 *
 * Create an i2c dev handle using the i2c bus handle 
 * 
 * 
 * 
 * @return N/A
 * @param dev_addr hex device address
 * @param bus_handle pointer to the i2c bus handle
 * @param dev_handle pointer to i2c device handle
 * @note if the bus initialization is unsuccessful, the system state is updated with a failure to add
 * i2c device
 * @bug Optional known bugs can be listed here.
 */

void i2c1_master_add_device(uint8_t dev_addr,
    i2c_master_dev_handle_t *dev_handle,
    i2c_master_bus_handle_t *bus_handle)
{
    if(bus_handle == NULL){
        //  2DO
        // 1. pull the system state,
        // 2. update the i2c bus status in sys state
        // 3. push the updated sys state to the global state
        return;
    }
    if(dev_handle == NULL){
        //  2DO
        // 1. pull the system state,
        // 2. update the i2c dev status in sys state
        // 3. push the updated sys state to the global state
        return;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C1_FREQ_HZ,
    };
    int retry_count = 0;
    while(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle) != ESP_OK && retry_count++ < 5){
    }
    
    if(retry_count >= 5) {
        //  2DO
        // 1. pull the system state,
        // 2. update the i2c dev status in sys state
        // 3. push the updated sys state to the global state
    }
}
