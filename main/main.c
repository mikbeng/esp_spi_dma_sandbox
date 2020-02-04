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

void app_main()
{
    esp_err_t ret;
    mspi_config_t mspi_config;
    mspi_dma_config_t mspi_dma_config;
    mspi_device_handle_t mspi_handle;

    uint16_t revol_reg;
    int8_t rev;
    uint8_t rx_data[4] = {0};
    uint32_t rx_length = 0;

    ESP_LOGI(__func__, "SPI EXPERIMENT DMA started");

    //Debug GPIO
    gpio_pad_select_gpio(GPIO_NUM_4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(GPIO_NUM_2);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);


    //Init mspi
    mspi_config.host			= HSPI_HOST;
    mspi_config.mosiGpioNum	    = PIN_NUM_MOSI;
    mspi_config.sckGpioNum      = PIN_NUM_CLK;
    mspi_config.csGpioNum       = PIN_NUM_CS;
    mspi_config.spi_clk         = 1000000;  //8Mhz
    
    mspi_init(&mspi_config, &mspi_handle);

    //Init DMA
    mspi_dma_config.dmaChan    = 1;
    mspi_dma_config.list_num   = 2;         //Number of linked lists to use (=number of buffers to use) 
    mspi_dma_config.dma_trans_len = 2;      //2 bytes
    mspi_dma_config.list_buf_size   = 4;    //Size of each buffer in list, must be word-aligned

    mspi_DMA_init(&mspi_dma_config, mspi_handle);

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

    mspi_set_addr(cmd, 16, 1, mspi_handle);  //Command word to TLE5012 in Address phase, will send MSB first if SPI_WR_BIT_ORDER = 0.
    mspi_set_miso(16,1, mspi_handle);

    //Kick off transfers
    mspi_start_continous_DMA_rx(mspi_handle);

    while(1)
    { 
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        mspi_get_dma_data_rx(&rx_data, &rx_length, mspi_handle);
        revol_reg = ((uint16_t)(rx_data[0]) << 8) | ((uint16_t)rx_data[1]);
        rev = (int8_t)(revol_reg & 0x01FF);

        ESP_LOGI(__func__, "revol_reg: %d ", revol_reg);
        ESP_LOGI(__func__, "rev: %d ", rev);
    }
}
