#include <stdio.h>
#include "SENSOR.h"
#include "driver/i2c_types.h"
#include "i2c1.h"
#include "ms5837.h"
#include "mpu9250.h"

#include "common_types.h"
#include "configuration.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys_common.h"

/*Constants*/
// #define DEBUG
#define TAG "Sensors"

/*Private variables*/
static sensor_data_t sensor_data = {0};
static i2c_master_bus_handle_t bus1;
static i2c_master_dev_handle_t imu1;
static i2c_master_dev_handle_t imu2;
static i2c_master_dev_handle_t ps;

static mpu9250_data_t imu1_data = {0};
static mpu9250_data_t imu2_data = {0};

#ifdef DEBUG
const static char *TAG = "SENSOR";
#endif

/*Private Functions*/
static void sensor_init(void);

static void sensor_init(void)
{
    system_status_t sys_status = {0};

    get_system_status(&sys_status);

    i2c1_master_init(&bus1);
    sys_status.i2c_bus_status = STATUS_OK;

    i2c1_master_add_device(MPU9250_I2C_ADDRESS0, &imu1, &bus1);
    i2c1_master_add_device(MPU9250_I2C_ADDRESS1, &imu2, &bus1);
    i2c1_master_add_device(ADDR_I2C_MS5837, &ps, &bus1);

    mpu9250_set_default_config(imu1);
    mpu9250_set_default_config(imu2);

    sys_status.ps_status = (bar30_setup(bus1, ps) == 0U) ? STATUS_OK : STATUS_ERROR;
    update_system_status(sys_status);
}

void SENSOR(void* params){
    uint8_t bar30_buffer[BAR30_READ_BUFFER_SIZE] = {0};
    float bar30_pressure = 0.0f;
    float bar30_temp = 0.0f;
    (void)params;

    sensor_init();

    // main sensor polling task
    for(;;){
        mpu9250_get_pose(imu1, &imu1_data);
        mpu9250_get_pose(imu2, &imu2_data);

        bar30_read(ps, bar30_buffer);

        sensor_data.imu1 = imu1_data;
        sensor_data.imu2 = imu2_data;
        memcpy(sensor_data.bar30_data, bar30_buffer, sizeof(bar30_buffer));
        
        // ESP_LOGI(TAG, "size: %d", sizeof(sensor_data));

        update_sensor_data(sensor_data);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
