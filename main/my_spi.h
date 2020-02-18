#ifndef MY_SPI_H
#define MY_SPI_H

#include <driver/spi_common.h>
#include <driver/spi_master.h>
//#include "my_config.h"
//#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_NUM_MOSI 5
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   27


typedef struct
{
    int				dmaChan;		        // 0 for no DMA, 1 or 2 for DMA
    uint32_t        list_num;               //Number of linked lists to use (=number of buffers to use) 
    uint32_t        dma_trans_len;          //The number of bytes actually written by DMA in each transfer (into each buffer). 
    bool            isrx;                   //True if DMA buffer is used as rx, false otherwise      
    bool            linked_list_circular;   //True if linked list is to be set up in a circular mode (last descriptor points to first). False otherwise.            
}mspi_dma_config_t;


typedef struct
{
    spi_host_device_t	host;			// HSPI_HOST or VSPI_HOST
    gpio_num_t			mosiGpioNum;	// GPIO MOSI
    gpio_num_t			sckGpioNum;	    // GPIO SCK
    gpio_num_t			csGpioNum;	    // GPIO CS
    double              spi_clk;        //SPI clock speed in Hz
    int                 dummy_cycle;
}mspi_config_t;

typedef struct
{
    int			    dmaChan;		// 0, 1 or 2
    lldesc_t*       descs;         //DMA Descriptors
    intr_handle_t   dma_intr;       //Interrupt handle for spi dma
    uint32_t        *dma_buffer;
    uint32_t         buffer_len;
}mspi_dma_handle_t;

typedef struct {
    spi_host_device_t	    host;			// HSPI_HOST or VSPI_HOST
    gpio_num_t			    mosiGpioNum;	// GPIO MOSI
    gpio_num_t			    sckGpioNum;	    // GPIO SCK
    gpio_num_t			    csGpioNum;	    // GPIO CS
    spi_dev_t*			    hw;
    intr_handle_t           trans_intr;           //Interrupt handle for spi 
    double                  clk_speed;
    int                     dummy_cycle;
    uint32_t                initiated;
    volatile uint32_t       transfer_cont;      //Flag for continuous transfer mode
    volatile uint32_t       polling_active;     //Flag for polling transfer mode
    volatile uint32_t       polling_done;       //Flag for polling transfer done
    mspi_dma_handle_t       dma_handle;
} spi_internal_t;

typedef struct
{
    uint8_t	            *txdata;			// Pointer to transmit buffer, or NULL for no MOSI phase
    uint8_t			    *rxdata;	        // Pointer to recieve buffer, or NULL for no MISO phase
    uint32_t 			tx_len;	            // Length of MOSI data in bits, 0 if no MOSI phase
    uint32_t 			rx_len;	            // Length of MISO data in bits, 0 if no MISO phase
    uint32_t            addr_len;           // Length of address phase in bits, 0 if no address phase
    uint32_t            cmd_len;            // Length of cmd phase in bits, 0 if no cmd phase
    uint32_t            addr;               // Value of transmitting address
    uint16_t            cmd;                // Value of transmitting command
}mspi_transaction_t;

typedef spi_internal_t* mspi_device_handle_t;  ///< Handle for a device on a SPI bus

esp_err_t mspi_init(mspi_config_t *mspi_config, mspi_device_handle_t* handle);
esp_err_t mspi_DMA_init(mspi_dma_config_t *mspi_dma_config, mspi_device_handle_t handle);
esp_err_t mspi_DMA_deinit(mspi_device_handle_t handle);
esp_err_t mspi_deinit(mspi_device_handle_t handle);
esp_err_t mspi_start_continuous_DMA(mspi_transaction_t *mspi_trans_p, mspi_device_handle_t handle);
esp_err_t mspi_stop_continuous_DMA(mspi_device_handle_t handle);


esp_err_t mspi_get_dma_data_rx(uint8_t *rxdata, uint32_t *rx_len_bytes, mspi_device_handle_t handle);

extern volatile uint32_t int_cnt;

#ifdef __cplusplus
} // extern "C"
#endif

#endif