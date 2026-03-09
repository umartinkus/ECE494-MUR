idf_component_register(SRCS "app.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES "freertos"
                                "SENSOR"
                                "HEALTH_MONITOR"
                                "COMMS")