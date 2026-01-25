idf_component_register(SRCS "THRUST_CTRL.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES pwm esp_driver_gpio)
