#pragma once
typedef enum I2C_ERR{
    I2C_OK = 0,
    I2C_BUS_NULL_PTR = 1,
    I2C_DEV_NULL_PTR = 2,
    I2C_BUS_INIT_FAIL = 3,
    I2C_DEV_ADD_FAIL = 4,
    I2C_NOT_INIT = 5,
    I2C_UNKNOWN_ERR = 99
} I2C_ERR_t;

typedef enum SPI_ERR_t{
    SPI_OK = 0,
    SPI_BUS_NULL_PTR = 1,
    SPI_DEV_NULL_PTR = 2, //dont think i need
    SPI_BUS_INIT_FAIL = 3,
    SPI_DEV_ADD_FAIL = 4, // likely dont need
    SPI_NOT_INIT = 5,
    SPI_UNKNOWN_ERR = 99
} SPI_ERR_t;