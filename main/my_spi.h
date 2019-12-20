#ifndef MY_SPI_H
#define MY_SPI_H

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include "my_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MY_ESP_ERR_SPI(x)                       MY_ESP_ERR(MY_ESP_ERR_SPI_BASE, (x))
#define MY_ESP_ERR_SPI_SPI1_IS_NOT_SUPPORTED    MY_ESP_ERR_SPI(1)
#define MY_ESP_ERR_SPI_INVALID_HOST_NUMBER      MY_ESP_ERR_SPI(2)
#define MY_ESP_ERR_SPI_INVALID_DMA_CHANNEL      MY_ESP_ERR_SPI(3)
#define MY_ESP_ERR_SPI_HOST_ALREADY_IN_USE      MY_ESP_ERR_SPI(4)
#define MY_ESP_ERR_SPI_DMA_ALREADY_IN_USE       MY_ESP_ERR_SPI(5)

#define PIN_NUM_MOSI 5
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   27

typedef struct
{
    spi_host_device_t	host;			// HSPI_HOST or VSPI_HOST
    int					dmaChan;		// 0, 1 or 2
    gpio_num_t			mosiGpioNum;	// GPIO MOSI
    gpio_num_t			sckGpioNum;	    // GPIO SCK
    gpio_num_t			csGpioNum;	    // GPIO CS
    double              spi_clk         //SPI clock speed in Hz
}my_spi_config_t;


esp_err_t myspi_init(my_spi_config_t *my_spi_config);
esp_err_t myspi_deinit(my_spi_config_t *my_spi_config);
esp_err_t myspi_start_tx_transfers(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif