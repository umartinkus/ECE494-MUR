idf_component_register(SRCS "UART_LISTEN.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_uart freertos)
