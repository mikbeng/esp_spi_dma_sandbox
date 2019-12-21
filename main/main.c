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

void app_main()
{
    esp_err_t ret;
    my_spi_config_t my_spi_config;

    ESP_LOGI(__func__, "SPI EXPERIMENT DMA started");

    gpio_pad_select_gpio(GPIO_NUM_23);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);


    //Init myspi
    my_spi_config.host			= HSPI_HOST;
    my_spi_config.dmaChan		= 1;
    my_spi_config.mosiGpioNum	= PIN_NUM_MOSI;
    my_spi_config.sckGpioNum    = PIN_NUM_CLK;
    my_spi_config.csGpioNum     = PIN_NUM_CS;
    my_spi_config.spi_clk       = 1000000;  //8Mhz
    
    myspi_init(&my_spi_config);

    //Set up tx buffer
    uint32_t *spi_tx_buf;
    uint16_t len_word = 10;
    uint16_t len_bytes = sizeof(uint32_t) * len_word;

    ESP_LOGI(__func__, "Allocating tx buffer with %d bytes", len_bytes);
    spi_tx_buf = (uint32_t *)heap_caps_malloc(len_bytes, MALLOC_CAP_DMA);       	//For DMA

    *spi_tx_buf = 0x44332211;
    *(spi_tx_buf+1) = 0x88776655;

    myspi_DMA_init(my_spi_config.host, my_spi_config.dmaChan, (void *)spi_tx_buf);

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


    //Kick off tx transfers
    myspi_start_tx_transfers();

    while(1)
    {   
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
