#include "spi_setup.h"
#include "../../common/configuration.h"

static spi_host_device_t s_host = SPI3_HOST;

esp_err_t spi3_slave_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI3_MOSI_PIN,
        .miso_io_num = SPI3_MISO_PIN,
        .sclk_io_num = SPI3_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = SPI3_MAX_TRANSFER_SZ,
        .flags = SPICOMMON_BUSFLAG_SLAVE,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    };

    spi_slave_interface_config_t slave_cfg = {
        .mode = SPI3_MODE,
        .spics_io_num = SPI3_CS_PIN,
        .queue_size = SPI3_SLAVE_QUEUE_SIZE,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };

    return spi_slave_initialize(s_host, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO);
}

esp_err_t spi3_slave_deinit(void)
{
    return spi_slave_free(s_host);
}