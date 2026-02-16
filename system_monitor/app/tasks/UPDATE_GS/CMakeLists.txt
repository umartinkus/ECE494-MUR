idf_component_register(SRCS "UPDATE_GS.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES freertos dataPacket esp_driver_uart ms5837 esp_driver_i2c)
