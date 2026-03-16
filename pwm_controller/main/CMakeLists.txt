idf_component_register(SRCS "pwm_controller.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos SENSOR COMMS THRUST_CTRL sys_common)
