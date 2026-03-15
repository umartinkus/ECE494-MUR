idf_component_register(SRCS "THRUST_CTRL.c"
                    INCLUDE_DIRS "include"
                    REQUIRES freertos pwm COMMS sys_common)
