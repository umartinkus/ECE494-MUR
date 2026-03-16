idf_component_register(SRCS "COMMS.c" "spi.c" "packet.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_spi freertos sys_common)
