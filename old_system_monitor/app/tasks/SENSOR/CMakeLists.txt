idf_component_register(SRCS "SENSOR.c" "i2c1_setup.c"
                    INCLUDE_DIRS "include" 
                    REQUIRES esp_driver_i2c sys_common)
