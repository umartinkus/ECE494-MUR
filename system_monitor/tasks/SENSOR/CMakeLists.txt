idf_component_register(SRCS "SENSOR.c" "i2c1.c"
                    INCLUDE_DIRS "include"
                    REQUIRES sys_common esp_driver_i2c adc freertos)
