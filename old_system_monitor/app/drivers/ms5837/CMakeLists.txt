idf_component_register(SRCS "ms5837.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "freertos"
                                  "esp_driver_i2c")
