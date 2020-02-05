#ifndef MY_SPI_H
#define MY_SPI_H

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include "my_config.h"
#include "esp_err.h"

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
    int				dmaChan;		// 0 for no DMA, 1 or 2 for DMA
    uint32_t        list_num;       //Number of linked lists to use (=number of buffers to use) 
    uint32_t        list_buf_size;   //Size in bytes of each buffer in list, must be word-aligned
    uint32_t        dma_trans_len;  //The number of bytes actually written by DMA into each buffer.
                        
}mspi_dma_config_t;

typedef struct
{
    spi_host_device_t	host;			// HSPI_HOST or VSPI_HOST
    gpio_num_t			mosiGpioNum;	// GPIO MOSI
    gpio_num_t			sckGpioNum;	    // GPIO SCK
    gpio_num_t			csGpioNum;	    // GPIO CS
    double              spi_clk;        //SPI clock speed in Hz
}mspi_config_t;


typedef struct {
    spi_host_device_t	    host;			// HSPI_HOST or VSPI_HOST
    int					    dmaChan;		// 0, 1 or 2
    gpio_num_t			    mosiGpioNum;	// GPIO MOSI
    gpio_num_t			    sckGpioNum;	    // GPIO SCK
    gpio_num_t			    csGpioNum;	    // GPIO CS
    spi_dev_t*			    hw;
    lldesc_t*               descs;         //DMA Descriptor
    intr_handle_t           trans_intr;           //Interrupt handle for spi 
    intr_handle_t           dma_intr;       //Interrupt handle for spi dma
    uint32_t                *dma_buffer;
    double                  clk_speed;
    int                     dummy_cycle;
    uint32_t                initiated;
    volatile uint32_t       transfer_cont;
    volatile uint32_t       transfer_done;
} spi_internal_t;

typedef spi_internal_t* mspi_device_handle_t;  ///< Handle for a device on a SPI bus

esp_err_t mspi_init(mspi_config_t *mspi_config, mspi_device_handle_t* handle);
esp_err_t mspi_DMA_init(mspi_dma_config_t *mspi_dma_config, mspi_device_handle_t handle);
esp_err_t mspi_deinit(mspi_device_handle_t handle);
esp_err_t mspi_start_continuous_DMA_rx(mspi_device_handle_t handle);
esp_err_t mspi_stop_continuous_DMA_rx(mspi_device_handle_t handle);

esp_err_t mspi_set_addr(uint32_t addr, uint32_t len, bool enable, mspi_device_handle_t handle);
esp_err_t mspi_set_mosi(uint32_t len, bool enable, mspi_device_handle_t handle);
esp_err_t mspi_set_miso(uint32_t len, bool enable, mspi_device_handle_t handle);
esp_err_t mspi_get_dma_data_rx(uint8_t *rxdata, uint32_t *rx_len_bytes, mspi_device_handle_t handle);

extern volatile int int_cnt;

#ifdef __cplusplus
} // extern "C"
#endif

#endif