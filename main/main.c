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
#include "my_spi.h"k

uint32_t *spi_rx_buf;

void app_main()
{
    esp_err_t ret;
    my_spi_config_t my_spi_config;

    uint16_t revol_reg;
    int8_t rev;
    uint8_t rx_data[4] = {0};

    ESP_LOGI(__func__, "SPI EXPERIMENT DMA started");

    //Debug GPIO
    gpio_pad_select_gpio(GPIO_NUM_4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(GPIO_NUM_2);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);


    //Init mspi
    my_spi_config.host			= HSPI_HOST;
    my_spi_config.dmaChan		= 1;
    my_spi_config.mosiGpioNum	= PIN_NUM_MOSI;
    my_spi_config.sckGpioNum    = PIN_NUM_CLK;
    my_spi_config.csGpioNum     = PIN_NUM_CS;
    my_spi_config.spi_clk       = 1000000;  //8Mhz
    
    mspi_init(&my_spi_config);

    //Set up tx buffer
    uint16_t len_word = 10;
    uint16_t len_bytes = sizeof(uint32_t) * len_word;

    ESP_LOGI(__func__, "Allocating tx buffer with %d bytes", len_bytes);
    spi_rx_buf = (uint32_t *)heap_caps_malloc(len_bytes, MALLOC_CAP_DMA);       	//For DMA

    ESP_LOGI(__func__, "Successfully allocates spi_buffer on address: %p",spi_rx_buf);

    mspi_DMA_init(my_spi_config.host, my_spi_config.dmaChan, spi_rx_buf);

    //prep command word
    uint8_t TLE5012B_cmd_RW = 1;		//read operation
    uint8_t address = 0x04;
    uint8_t TLE5012B_cmd_LOCK = 0b0000;
	if (address >= 0x05 && address <= 0x11)
	{
		uint8_t TLE5012B_cmd_LOCK = 0b1010;    	//Configuration access for addresses 0x05 : 0x11
	}

	uint8_t TLE5012B_cmd_UPD = 0;
	uint8_t TLE5012B_cmd_ADDR = address;
	uint8_t TLE5012B_cmd_ND = 0;				//0x01 for Safety word. 0x00 for no Safety word
    uint16_t cmd = ((TLE5012B_cmd_RW << 15) | (TLE5012B_cmd_LOCK << 11) | (TLE5012B_cmd_UPD << 10) | (TLE5012B_cmd_ADDR << 4) | (TLE5012B_cmd_ND << 0));

    mspi_set_addr(cmd, 16, 1);  //Command word to TLE5012 in Address phase, will send MSB first if SPI_WR_BIT_ORDER = 0.
    mspi_set_miso(16,1);

    //Kick off transfers
    mspi_start_transfers();

    while(1)
    { 
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        mspi_get_dma_data_rx(&rx_data, 2);
        revol_reg = ((uint16_t)(rx_data[0]) << 8) | ((uint16_t)rx_data[1]);
        rev = (int8_t)(revol_reg & 0x01FF);

        ESP_LOGI(__func__, "revol_reg: %d ", revol_reg);
        ESP_LOGI(__func__, "rev: %d ", rev);
    }
}
