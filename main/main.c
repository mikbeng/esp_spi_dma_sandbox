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
#include "soc/spi_struct.h"

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
    intr_handle_t intr;
    intr_handle_t intr_dma;
} spi;

const int		SpiHSyncBackporchWaitCycle	= 0;//58;		// H-Sync back porch : 2.20us

#define in_suc_eof_int_en BIT(5)
#define out_eof_int_en BIT(7)
#define spi_trans_done_int BIT(4)


volatile int cnt = 0;
int level = 0;

// This is run in interrupt context.
void spi_intr_dma(void)
{
    uint32_t spi_intr_status;
    uint32_t spi_intr_raw;

    spi_intr_raw = spi.hw->dma_int_raw.val;

    spi_intr_status = spi.hw->dma_int_st.val; //Read interrupt status

    //spi_intr_status = spi.hw->slave.trans_done; //Read interrupt status

    //ets_printf("INT! spi_intr_status:%d\n", spi_intr_status);
    //ets_printf("INT! spi_intr_raw:%d\n", spi_intr_raw);
    
    gpio_set_level(GPIO_NUM_23, level);
    level ^= 1;

    if (spi_intr_status & out_eof_int_en) { //Check for interrupt on rising edge on CAP0 signal

        //ets_printf("INT! spi_intr_status:%d\n", spi_intr_status);
        //ets_printf("INT! spi_intr_raw:%d\n", spi_intr_raw);
        //spi.hw->dma_in_link.start   	= 0;
        //spi.hw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)
        //spi.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer
    }

    spi.hw->dma_int_clr.val = spi_intr_status;     //Clears the interrupt
}

// This is run in interrupt context.
void spi_intr(void)
{
    esp_intr_disable(spi.intr);
    uint32_t spi_intr_slave_val;

    spi_intr_slave_val = spi.hw->slave.val; //Read interrupt status

    //spi_intr_status = spi.hw->slave.trans_done; //Read interrupt status

    //ets_printf("INT! spi_intr_slave_val:%d\n", spi_intr_slave_val);
    //ets_printf("INT! spi_intr_raw:%d\n", spi_intr_raw);
    
    gpio_set_level(GPIO_NUM_23, level);
    level ^= 1;

    if (spi_intr_slave_val & spi_trans_done_int) { //Check for interrupt on rising edge on CAP0 signal

        //ets_printf("INT! spi_intr_status:%d\n", spi_intr_status);
        //ets_printf("INT! spi_intr_raw:%d\n", spi_intr_raw);
        //spi.hw->dma_in_link.start   	= 0;
        spi.hw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)
        spi.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer
    }

    spi.hw->slave.trans_done = 0;     //Clears the interrupt
    esp_intr_enable(spi.intr);
}

void app_main()
{

    ESP_LOGI(__func__, "SPI EXPERIMENT DMA started");


    gpio_pad_select_gpio(GPIO_NUM_23);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);

    esp_err_t ret;

    spi.host			= HSPI_HOST;
    spi.dmaChan			= 1;
    spi.mosiGpioNum		= PIN_NUM_MOSI;
    spi.sckGpioNum      = PIN_NUM_CLK;
    spi.csGpioNum       = PIN_NUM_CS;
    spi.hw				= myspi_get_hw_for_host(HSPI_HOST);
    spi.descs = (lldesc_t *)calloc(2, sizeof(lldesc_t));


    const double SpiDmaClockSpeedInHz = 8000000;    //8Mhz

    uint32_t *spi_rx_buf;
    uint16_t len_word = 10;
    uint16_t len_bytes = sizeof(uint32_t) * len_word;


    //prep command word
    uint8_t TLE5012B_cmd_RW = 0;		//Write operation
    uint8_t address = 0x00;
    uint8_t TLE5012B_cmd_LOCK = 0b0000;
	if (address >= 0x05 && address <= 0x11)
	{
		uint8_t TLE5012B_cmd_LOCK = 0b1010;    	//Configuration access for addresses 0x05 : 0x11
	}

	uint8_t TLE5012B_cmd_UPD = 0;
	uint8_t TLE5012B_cmd_ADDR = address;
	uint8_t TLE5012B_cmd_ND = 0;					//No safety word for write
    uint16_t cmd = ((TLE5012B_cmd_RW << 15) | (TLE5012B_cmd_LOCK << 11) | (TLE5012B_cmd_UPD << 10) | (TLE5012B_cmd_ADDR << 4) | (TLE5012B_cmd_ND << 0));

    ESP_LOGI(__func__, "Allocating tx buffer with %d bytes", len_bytes);
    spi_rx_buf = (uint32_t *)heap_caps_malloc(len_bytes, MALLOC_CAP_DMA);       	//For DMA

    *spi_rx_buf = 0x44332211;
    *(spi_rx_buf+1) = 0x88776655;

    //Configure DMA link
    spi.descs->owner = 1;
    spi.descs->eof = 1;
    spi.descs->length = 4;
    spi.descs->size = 4;
    spi.descs->qe.stqe_next = spi.descs+1;
    spi.descs->buf = spi_rx_buf;


    spi.descs[1].owner = 1;
    spi.descs[1].eof = 1;
    spi.descs[1].length = 4;
    spi.descs[1].size = 4;
    spi.descs[1].qe.stqe_next    = spi.descs;
    spi.descs[1].buf = spi_rx_buf+1;


    ret = myspi_prepare_circular_buffer(
            spi.host
        , spi.dmaChan
        , spi.descs
        , SpiDmaClockSpeedInHz
        , spi.mosiGpioNum
        , spi.sckGpioNum
        , spi.csGpioNum
        , SpiHSyncBackporchWaitCycle
    );
    if (ret != ESP_OK) {
        ESP_LOGE(__func__, "myspi_prepare_circular_buffer() returned %d", ret);
    }

    //spi.hw->slave.trans_inten = 1;
    //spi.hw->addr                        = cmd;    //Address test. Will send MSB first if SPI_WR_BIT_ORDER = 0.
    
    /*
    spi.hw->dma_int_ena.out_eof = 1;
    int flags = 0;
    ret = esp_intr_alloc(ETS_SPI2_DMA_INTR_SOURCE, flags, &spi_intr_dma, NULL, &spi.intr_dma);
    if (ret != ESP_OK) {
        ESP_LOGE(__func__, "esp_intr_alloc() returned %d", ret);
    }
    //esp_intr_enable(spi.intr);
    int int_cpu = esp_intr_get_cpu(spi.intr_dma);
    ESP_LOGI(__func__, "Allocated interrupt on cpu %d", int_cpu);
    */

    spi.hw->slave.trans_inten = 1;
    int flags = 0;
    ret = esp_intr_alloc(ETS_SPI2_INTR_SOURCE, flags, &spi_intr, NULL, &spi.intr);
    if (ret != ESP_OK) {
        ESP_LOGE(__func__, "esp_intr_alloc() returned %d", ret);
    }


    spi.hw->dma_conf.dma_tx_stop		= 1;	// Stop SPI DMA

    //spi.hw->dma_conf.dma_rx_stop		= 1;	// Stop SPI DMA
    //spi.hw->ctrl2.val           		= 0;	// Reset timing
    //spi.hw->dma_conf.dma_rx_stop		= 0;	// Disable stop

    spi.hw->dma_conf.dma_tx_stop		= 0;	// Disable stop
    //spi.hw->dma_conf.dma_continue	= 1;	// Set contiguous mode
    //spi.hw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)

    //spi.hw->dma_in_link.start   		= 1;
    //spi.h->cmd.usr					= 1;	// SPI: Start SPI DMA transfer

    //portENABLE_INTERRUPTS();
    //esp_intr_disable(spi.intr_dma);
    esp_intr_enable(spi.intr);


    spi.hw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)
    spi.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer

    while(1)
    {   
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(__func__, "In while loop. cnt=%d", cnt);
        //spi.hw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)
        //spi.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer
        //spi.hw->dma_in_link.start   		= 1;
        //spi.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer
    }
}
