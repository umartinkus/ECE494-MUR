idf_component_register(SRCS "GET_POSE.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES mpu9250 freertos dataPacket)
