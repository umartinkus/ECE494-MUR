idf_component_register(SRCS "COMMS.c" "spi_setup.c"
                    INCLUDE_DIRS "include"
                    REQUIRES freertos esp_driver_spi sys_common)
