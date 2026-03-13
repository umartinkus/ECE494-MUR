idf_component_register(SRCS "system_monitor.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos SENSOR COMMS)
