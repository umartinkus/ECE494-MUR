idf_component_register(SRCS "pwm_controller.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos SENSOR COMMS PARSER THRUST_CTRL sys_common)
