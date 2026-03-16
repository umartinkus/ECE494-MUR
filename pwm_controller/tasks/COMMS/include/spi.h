#pragma once
#include "esp_err.h"
#include "driver/spi_slave.h"

esp_err_t spi3_slave_init(void);
esp_err_t spi3_slave_deinit(void);
esp_err_t spi_transaction(uint8_t* trans, uint8_t* receive, size_t len);
