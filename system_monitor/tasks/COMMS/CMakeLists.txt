idf_component_register(SRCS "COMMS.c" "spi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_spi sys_common freertos sys_common)
