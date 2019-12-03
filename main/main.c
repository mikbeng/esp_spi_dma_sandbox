/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include <soc/spi_reg.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include "my_spi.h"
#include <soc/dport_reg.h>

#define PIN_NUM_MOSI 5
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   27

struct {
    spi_host_device_t	host;			// HSPI_HOST or VSPI_HOST
    int					dmaChan;		// 0, 1 or 2
    gpio_num_t			mosiGpioNum;	// GPIO MOSI
    gpio_num_t			sckGpioNum;	    // GPIO SCK
    gpio_num_t			csGpioNum;	    // GPIO SCK
    spi_dev_t*			hw;
    lldesc_t *descs;
} spi;

const int		SpiHSyncBackporchWaitCycle	= 0;//58;		// H-Sync back porch : 2.20us

void app_main()
{

    ESP_LOGI(__func__, "SPI EXPERIMENT DMA started");

    spi.host			= HSPI_HOST;
    spi.dmaChan			= 1;
    spi.mosiGpioNum		= PIN_NUM_MOSI;
    spi.sckGpioNum      = PIN_NUM_CLK;
    spi.csGpioNum       = PIN_NUM_CS;
    spi.hw				= myspi_get_hw_for_host(HSPI_HOST);
    spi.descs = (lldesc_t *)calloc(2, sizeof(lldesc_t));


    const double SpiDmaClockSpeedInHz = 1000000;    //8Mhz

    uint32_t *spi_tx_buf;
    uint16_t len_word = 10;
    uint16_t len_bytes = sizeof(uint32_t) * len_word;

    ESP_LOGI(__func__, "Allocating tx buffer with %d bytes", len_bytes);
    spi_tx_buf = (uint32_t *)heap_caps_malloc(len_bytes, MALLOC_CAP_DMA);       	//For DMA

    spi_tx_buf[0] = 0x44332211;
    spi_tx_buf[1] = 0x88776655;
    //spi_tx_buf[1] = 0x43;
    /*
    uint8_t cnt = 1;
    for (size_t i = 0; i < len_word; i++)
    {
        spi_tx_buf[i] = cnt;
        ESP_LOGI(__func__, "buf:%d", spi_tx_buf[i]);
        cnt++;
    }*/
    
    //lldesc_t *dd = &spi.descs[0];

    //Configure DMA link
    spi.descs[0].owner = 1;
    spi.descs[0].eof = 0;
    spi.descs[0].length = 2;
    spi.descs[0].size = 4;
    spi.descs[0].qe.stqe_next    = &spi.descs[1];
    spi.descs[0].buf = spi_tx_buf;

    spi.descs[1].owner = 1;
    spi.descs[1].eof = 0;
    spi.descs[1].length = 2;
    spi.descs[1].size = 4;
    spi.descs[1].qe.stqe_next    = &spi.descs[0];
    spi.descs[1].buf = spi_tx_buf+1;


    myspi_prepare_circular_buffer(
            spi.host
        , spi.dmaChan
        , spi.descs
        , SpiDmaClockSpeedInHz
        , spi.mosiGpioNum
        , spi.sckGpioNum
        , spi.csGpioNum
        , SpiHSyncBackporchWaitCycle
    );

    portDISABLE_INTERRUPTS();

    spi.hw->dma_conf.dma_tx_stop		= 1;	// Stop SPI DMA
    spi.hw->ctrl2.val           		= 0;	// Reset timing
    spi.hw->dma_conf.dma_tx_stop		= 0;	// Disable stop
    //spi.hw->dma_conf.dma_continue	= 1;	// Set contiguous mode
    spi.hw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)


    spi.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer

    portENABLE_INTERRUPTS();

    while(1)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        spi.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer
    }
}
