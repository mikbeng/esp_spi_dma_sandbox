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

#include "string.h"
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include "mspi.h"

int64_t start_time, end_time;;
spi_device_handle_t spi_idf;

uint32_t level = 0;

uint16_t TLE5012B_get_cmd(uint8_t reg_address, uint8_t reg_num, bool safety_word){
    //prep command word
    uint8_t TLE5012B_cmd_RW = 1;		//read operation
    uint8_t address = reg_address;
    uint8_t TLE5012B_cmd_LOCK = 0b0000;
	if (address >= 0x05 && address <= 0x11)
	{
		uint8_t TLE5012B_cmd_LOCK = 0b1010;    	//Configuration access for addresses 0x05 : 0x11
	}

	uint8_t TLE5012B_cmd_UPD = 0;
	uint8_t TLE5012B_cmd_ADDR = address;

    uint8_t TLE5012B_cmd_ND = 0;
    if(!safety_word && reg_num == 1){
        TLE5012B_cmd_ND = 0;				    //0x00 for no Safety word (This is the way. For DMA to work..)
    }
    else
    {
        TLE5012B_cmd_ND = reg_num;				
    }
    
    uint16_t cmd = ((TLE5012B_cmd_RW << 15) | (TLE5012B_cmd_LOCK << 11) | (TLE5012B_cmd_UPD << 10) | (TLE5012B_cmd_ADDR << 4) | (TLE5012B_cmd_ND << 0));

    return cmd;
 
}

float TLE5012B_calc_angle_deg(uint16_t AVAL_reg)
{
	uint16_t temp_aval = 0;
	float ret_angle = 0;
		
    //temp_aval = ((int16_t)(angle_reg << 1)) >> 1;		//For angles (-180 ... +179.9)	Note that absolute angle calc wont work with this as it is now 
    temp_aval = (AVAL_reg & 0x7FFF);  					//For angles (0 ... +359.9) 
    
    ret_angle = (float)temp_aval * (360 / 32768.0);
    return ret_angle;

}
static esp_err_t IRAM_ATTR spi_idf_TLE5012B_poll(uint8_t address, uint16_t *reg_data, uint16_t *safe_word)
{
		esp_err_t ret;
		static spi_transaction_t t;
		uint32_t rx_swapped = 0;
		
		memset(&t, 0, sizeof(t));       	//Zero out the transaction
		t.addr = (uint64_t)TLE5012B_get_cmd(0x02, 1, true);
        //t.cmd = ((TLE5012B_cmd_now.RW << 15) | (TLE5012B_cmd_now.LOCK << 11) | (TLE5012B_cmd_now.UPD << 10) | (TLE5012B_cmd_now.ADDR << 4) | (TLE5012B_cmd_now.ND << 0));               
		t.length = 2*16;
		t.rxlength = 2*16;		//16 bit register + 16bit safety word
		t.flags = SPI_TRANS_USE_RXDATA;
		
        start_time = esp_timer_get_time();

        spi_device_polling_transmit(spi_idf, &t);

        end_time = esp_timer_get_time();


		ESP_LOGI(__func__, "spi-idf_polling time %lld", end_time-start_time);

		rx_swapped = SPI_SWAP_DATA_RX(*(uint32_t*)t.rx_data, 32);
		*reg_data = (uint16_t)(rx_swapped >> 16);
		*safe_word = (uint16_t)(rx_swapped & 0x0000FFFF);

		return ESP_OK;
		
}

static void spi_idf_init()
{
    esp_err_t ret;
    //SPI-idf test
    spi_bus_config_t buscfg =  {
		.miso_io_num = -1,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};
	
	//New config parameters for DMA
	spi_device_interface_config_t devcfg =  {
		.command_bits = 0,
		.address_bits = 16,
		.dummy_bits = 0,
		.clock_speed_hz = 8000000, 	//Clock out at 6 MHz				
		.mode = 1,					//SPI mode 1		                               
		.spics_io_num = PIN_NUM_CS,			//-1 if manually toggle CS 			
		.queue_size = 1,			//We want to be able to queue 4 transactions at a time        
		.cs_ena_posttrans = 2,		//CS hold post trans in us
		.cs_ena_pretrans = 2,		//CS setup time in us
		.flags = (SPI_DEVICE_HALFDUPLEX) | (SPI_DEVICE_3WIRE)
		//.input_delay_ns = 30		
		//.pre_cb = pre_cb_func,		//pre interrupt
	    //.post_cb = s_TLE5012B_transmission_done_cb  		//post interrupt 
	};
		
	//Initialize the SPI bus
	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
	if (ret != ESP_OK)
	{
		ESP_LOGE(__func__, "spi_bus_initialize(): returned %d", ret);
	}
	assert(ret == ESP_OK);
	
	//Attach the device to the SPI bus
	ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_idf);
	if (ret != ESP_OK)
	{
		ESP_LOGE(__func__, "spi_bus_add_device(): returned %d", ret);
	}
	assert(ret == ESP_OK);

    spi_device_acquire_bus(spi_idf,portMAX_DELAY);

}

static void spi_task(void *arg)
{
    mspi_config_t mspi_config;
    mspi_dma_config_t mspi_dma_config;

    mspi_device_handle_t mspi_handle;
    mspi_dma_handle_t mspi_dma_handle;

    mspi_transaction_t spi_trans_aval;
    mspi_transaction_t spi_trans_revol;

    //Debug GPIO
    gpio_pad_select_gpio(GPIO_NUM_4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 0);

    gpio_pad_select_gpio(GPIO_NUM_2);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);
    
    uint8_t rx_data[4] = {0};
    uint32_t rx_length = 0;
    uint16_t AVAL_reg, REVOL_reg;
    uint16_t safe_w = 0;
    float angle;
    uint16_t revol;

    uint32_t TLE5012B_rx_words = 1;

    //Init mspi
    mspi_config.host			= HSPI_HOST;
    mspi_config.mosiGpioNum	    = PIN_NUM_MOSI;
    mspi_config.sckGpioNum      = PIN_NUM_CLK;
    mspi_config.csGpioNum       = PIN_NUM_CS;
    mspi_config.spi_clk         = 1000000;  //8Mhz
    mspi_config.dummy_cycle     = 3;        //This gives approximatly 200ns dummy phase, which should be enough for twr_delay according to datasheet 
    
    mspi_init(&mspi_config, &mspi_handle);

    //Init DMA
    mspi_dma_config.dmaChan    = 1;                 //dma channel 1
    mspi_dma_config.list_num   = 10;                 //Number of linked lists 2 
    mspi_dma_config.dma_trans_len = 2;              //2 bytes (16 bits rx data)
    mspi_dma_config.isrx        = true;             //DMA is set up for rx data
    mspi_dma_config.linked_list_circular = true;    //Linked list is circular

    mspi_DMA_init(&mspi_dma_config, mspi_handle);

    uint16_t cmd_aval = TLE5012B_get_cmd(0x02, 1, false); 
    uint16_t cmd_revol = TLE5012B_get_cmd(0x04, 1, false);

    //Zero out the transaction
    memset(&spi_trans_aval, 0, sizeof(spi_trans_aval));       	
    spi_trans_aval.addr_len = 16;
    spi_trans_aval.addr = cmd_aval;
    spi_trans_aval.rx_len = 16;
    spi_trans_aval.rxdata = &rx_data;

    //Zero out the transaction
    memset(&spi_trans_revol, 0, sizeof(spi_trans_revol));       	
    spi_trans_revol.addr_len = 16;
    spi_trans_revol.addr = cmd_revol;
    spi_trans_revol.rx_len = 16;
    spi_trans_revol.rxdata = &rx_data;

    // //Kick off transfers
    // mspi_start_continuous_DMA(&spi_trans_aval, mspi_handle);
    
    mspi_start_continuous_DMA_interrupt(&spi_trans_aval, mspi_handle);

    //spi_idf_init();
    //ESP_LOGI(__func__, "SPI IDF initialized!");

    int loop_cnt = 0;

    while(1)
    { 
        vTaskDelay((10) / portTICK_PERIOD_MS);

        // mspi_device_transfer_blocking(&spi_trans_aval, mspi_handle);
        // AVAL_reg = ((uint16_t)(spi_trans_aval.rxdata[0]) << 8) | ((uint16_t)spi_trans_aval.rxdata[1]);
        // angle = TLE5012B_calc_angle_deg(AVAL_reg);
        // ESP_LOGI(__func__, "angle: %.2f ", angle);

        //spi_idf_TLE5012B_poll(0x02, &AVAL_reg, &safe_w);

        mspi_get_dma_data_rx(&spi_trans_aval, mspi_handle);
        AVAL_reg = ((uint16_t)(spi_trans_aval.rxdata[0]) << 8) | ((uint16_t)spi_trans_aval.rxdata[1]);
        angle = TLE5012B_calc_angle_deg(AVAL_reg);
        //ESP_LOGI(__func__, "angle: %.2f, interrupt_time: %lld ", angle, time_delta);


        loop_cnt++;
        if(loop_cnt == 50){

            ESP_LOGI(__func__, "angle: %.2f ", angle);
            //ESP_LOGI(__func__, "Stopping DMA");
            vTaskSuspendAll();
            start_time = esp_timer_get_time();
            mspi_stop_continuous_DMA_interrupt(mspi_handle);
            //memset(&rx_data,0,sizeof(rx_data));

            mspi_device_transfer_blocking(&spi_trans_revol, mspi_handle);
            REVOL_reg = ((uint16_t)(spi_trans_revol.rxdata[0]) << 8) | ((uint16_t)spi_trans_revol.rxdata[1]);
            revol = (((int16_t)(REVOL_reg << 7)) >> 7);
            //ESP_LOGI(__func__, "Starting DMA");

            mspi_start_continuous_DMA_interrupt(&spi_trans_aval, mspi_handle);
            end_time = esp_timer_get_time();
            loop_cnt = 0;
            xTaskResumeAll();

            ESP_LOGI(__func__, "delta-time: %lld", end_time-start_time);
            ESP_LOGI(__func__, "revol: %d ", revol);
        }

    }
}

void app_main()
{
    esp_err_t ret;
   
    ESP_LOGI(__func__, "SPI EXPERIMENT DMA started");
    
    xTaskCreatePinnedToCore(spi_task, "stats", 4096, NULL, 3, NULL, 1);

    while(1)
    { 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
