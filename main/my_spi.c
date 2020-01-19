/* ========================================================================= */
/* [INCL] Includes                                                           */
/* ========================================================================= */
#include <soc/spi_reg.h>
#include <soc/dport_reg.h>
#include "my_spi.h"
#include "esp_log.h"
#include <soc/dport_reg.h>
#include "soc/spi_struct.h"


/* ========================================================================= */
/* [DEFS] Defines                                                            */
/* ========================================================================= */
#define in_suc_eof_int_en BIT(5)
#define out_eof_int_en BIT(7)
#define spi_trans_done_int BIT(4)

/* ========================================================================= */
/* [TYPE] Type definitions                                                   */
/* ========================================================================= */

typedef struct {
    spi_host_device_t	    host;			// HSPI_HOST or VSPI_HOST
    int					    dmaChan;		// 0, 1 or 2
    gpio_num_t			    mosiGpioNum;	// GPIO MOSI
    gpio_num_t			    sckGpioNum;	    // GPIO SCK
    gpio_num_t			    csGpioNum;	    // GPIO CS
    spi_dev_t*			    hw;
    lldesc_t*               descs;         //DMA Descriptor
    lldesc_t*               last_inlink_desc_eof;
    intr_handle_t           intr;           //Interrupt handle for spi 
    intr_handle_t           intr_dma;       //Interrupt handle for spi dma
    double                  clk_speed;
    int                     dummy_cycle;
} spi_internal_t;

/* ========================================================================= */
/* [PFDE] Private functions declaration                                      */
/* ========================================================================= */

/* ========================================================================= */
/* [GLOB] Global variables                                                   */
/* ========================================================================= */
spi_internal_t spi_internal;
int level_dma = 1;
int level_spi = 1;
volatile int int_cnt = 0;
volatile uint16_t test1, test2;

	
/* ========================================================================= */
/* [PFUN] Private functions implementations                                  */
/* ========================================================================= */

// This is run in interrupt context.
static void IRAM_ATTR s_spi_intr_dma(void)
{
    //Temporarily disable interrupt 
    esp_intr_disable(spi_internal.intr_dma);

    uint32_t spi_intr_status;
    uint32_t spi_intr_raw;

    spi_intr_raw = spi_internal.hw->dma_int_raw.val;

    spi_intr_status = spi_internal.hw->dma_int_st.val; //Read interrupt status

    //GPIO debug pin
    gpio_set_level(GPIO_NUM_4, level_dma);
    level_dma ^= 1;

    //Debug prints
    //ets_printf("DMA INT! spi_intr_status:%d\n", spi_intr_status);
    //ets_printf("DMA INT! spi_intr_raw:%d\n", spi_intr_raw);
    
    if (spi_intr_status & out_eof_int_en) { 

    }
    else if(spi_intr_status & in_suc_eof_int_en) { 
        spi_internal.hw->dma_in_link.start          = 1;
    }

    spi_internal.hw->dma_int_clr.val = spi_intr_status;     //Clears the interrupt

    //Finally, enable the interrupt
    esp_intr_enable(spi_internal.intr_dma);
}

// This is run in interrupt context.
static void IRAM_ATTR s_spi_intr(void)
{
    //Temporarily disable interrupt 
    esp_intr_disable(spi_internal.intr);

    uint32_t spi_intr_slave_val;

    //GPIO debug pin
    gpio_set_level(GPIO_NUM_2, level_spi);
    level_spi ^= 1;

    spi_intr_slave_val = spi_internal.hw->slave.val; //Read interrupt status

    //ets_printf("SPI INT! spi_intr_slave_val:%d\n", spi_intr_slave_val);
    //ets_printf("SPI INT! spi_intr_raw:%d\n", spi_intr_raw);
    
    if (spi_intr_slave_val & spi_trans_done_int) { 

        /*
        ets_printf("SPI INT! dma_in_suc_eof_des_addr:0x%x\n", spi_internal.hw->dma_in_suc_eof_des_addr);
        ets_printf("SPI INT! dma_inlink_dscr:0x%x\n", spi_internal.hw->dma_inlink_dscr);
        ets_printf("SPI INT! dma_inlink_dscr_bf0:0x%x\n", spi_internal.hw->dma_inlink_dscr_bf0);
        ets_printf("SPI INT! dma_inlink_dscr_bf1:0x%x\n", spi_internal.hw->dma_inlink_dscr_bf1);
        */

        //ets_printf("SPI INT! *dma_data_p:0x%x\n", *dma_data_p);
        //ets_printf("SPI INT! rx_data:0x%x\n", rx_data);

        spi_internal.hw->dma_in_link.start          = 1;

        //Start new transfer
        spi_internal.hw->cmd.usr = 1;	            // SPI: Start new SPI transfer
        spi_internal.hw->slave.trans_done = 0;      //Clears the interrupt
    }
    
    //Finally, enable the interrupt
    esp_intr_enable(spi_internal.intr);
}

static spi_dev_t *myspi_get_hw_for_host(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return &SPI1; break;
    case HSPI_HOST: return &SPI2; break;
    case VSPI_HOST: return &SPI3; break;
    default:        return NULL;  break;
    }
}


static esp_err_t getSpidOutByHost(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_OUT_IDX;	break;
    case HSPI_HOST: return HSPID_OUT_IDX;	break;
    case VSPI_HOST: return VSPID_OUT_IDX;	break;
    default:        return ESP_FAIL;	break;
    }
}


static esp_err_t getSpidInByHost(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_IN_IDX;		break;
    case HSPI_HOST: return HSPID_IN_IDX;	break;
    case VSPI_HOST: return VSPID_IN_IDX;	break;
    default:        return ESP_FAIL;	break;
    }
}

static esp_err_t s_myspi_register_interrupt(spi_internal_t *spi)
{
    esp_err_t ret;

    //DMA interrupt
    /*
    spi->hw->dma_int_ena.in_suc_eof = 1;
    int flags = ESP_INTR_FLAG_IRAM;
    ret = esp_intr_alloc(ETS_SPI2_DMA_INTR_SOURCE, flags, &s_spi_intr_dma, NULL, &spi->intr_dma);
    if (ret != ESP_OK) {
        ESP_LOGE(__func__, "esp_intr_alloc() returned %d", ret);
    }
    //esp_intr_enable(spi.intr);
    int int_cpu = esp_intr_get_cpu(spi->intr_dma);
    ESP_LOGI(__func__, "Allocated interrupt on cpu %d", int_cpu);
    */

    //SPI interrupt
    spi->hw->slave.trans_inten = 1;
    int flags = ESP_INTR_FLAG_IRAM;
    ret = esp_intr_alloc(ETS_SPI2_INTR_SOURCE, flags, &s_spi_intr, NULL, &spi->intr);
    if (ret != ESP_OK) {
        ESP_LOGE(__func__, "esp_intr_alloc() returned %d", ret);
        return ESP_FAIL;
    }
    int int_cpu = esp_intr_get_cpu(spi->intr);
    ESP_LOGI(__func__, "Allocated interrupt ETS_SPI2_INTR_SOURCE on cpu %d", int_cpu);

    return ESP_OK;
}

static esp_err_t s_myspi_configure_clock(spi_internal_t *spi)
{
	// Set SPI Clock
	//  Register 7.7: SPI_CLOCK_REG (0x18)
	//
	//		SPI_CLK_EQU_SYSCLK
	//			In master mode, when this bit is set to 1, spi_clk is equal
	//			to system clock; when set to 0, spi_clk is divided from system
	//			clock.
	//
	//		SPI_CLKDIV_PRE
	//			In master mode, the value of this register field is the
	//			pre-divider value for spi_clk, minus one.
	//
	//		SPI_CLKCNT_N
	//			In master mode, this is the divider for spi_clk minus one.
	//			The spi_clk frequency is
	//				system_clock/(SPI_CLKDIV_PRE+1)/(SPI_CLKCNT_N+1).
	//
	//		SPI_CLKCNT_H
	//			For a 50% duty cycle, set this to floor((SPI_CLKCNT_N+1)/2-1)
	//
	//		SPI_CLKCNT_L
	//			In master mode, this must be equal to SPI_CLKCNT_N.

    const double	preDivider			= 2.0;
    const double	apbClockSpeedInHz	= APB_CLK_FREQ;
    const double	apbClockPerDmaCycle	= (apbClockSpeedInHz / preDivider / spi->clk_speed);

    const int32_t	clkdiv_pre	= ((int32_t) preDivider) - 1;
    const int32_t	clkcnt_n	= ((int32_t) apbClockPerDmaCycle) - 1;
    const int32_t	clkcnt_h	= (clkcnt_n + 1) / 2 - 1;
    const int32_t	clkcnt_l	= clkcnt_n;

    spi->hw->clock.clk_equ_sysclk	= 0;
    spi->hw->clock.clkcnt_n		    = clkcnt_n;
    spi->hw->clock.clkdiv_pre		= clkdiv_pre;
    spi->hw->clock.clkcnt_h		    = clkcnt_h;
    spi->hw->clock.clkcnt_l		    = clkcnt_l;

    return ESP_OK;
}

static esp_err_t s_myspi_configure_GPIO(spi_internal_t *spi)
{
    //Configure GPIOs for 3-wire half duplex (MOSI,SCK and CS)
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[spi->mosiGpioNum], PIN_FUNC_GPIO);
    gpio_set_direction(spi->mosiGpioNum, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(spi->mosiGpioNum, getSpidOutByHost(spi->host), false, false);
    gpio_matrix_in(spi->mosiGpioNum, getSpidInByHost(spi->host), false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[spi->sckGpioNum], PIN_FUNC_GPIO);
    gpio_set_direction(spi->sckGpioNum, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(spi->sckGpioNum, HSPICLK_OUT_IDX, false, false);
    gpio_matrix_in(spi->sckGpioNum, HSPICLK_IN_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[spi->csGpioNum], PIN_FUNC_GPIO);
    gpio_set_direction(spi->csGpioNum, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(spi->csGpioNum, HSPICS0_OUT_IDX, false, false);
    gpio_matrix_in(spi->csGpioNum, HSPICS0_IN_IDX, false);

    return ESP_OK;
}

static esp_err_t s_myspi_configure_registers(spi_internal_t *spi) 
{

    //Claim peripheral
    const bool spi_periph_claimed = spicommon_periph_claim(spi->host);
    if(! spi_periph_claimed) {
        return MY_ESP_ERR_SPI_HOST_ALREADY_IN_USE;
    }

    s_myspi_configure_GPIO(spi);

    //Reset timing
    spi->hw->ctrl2.val           		= 0;

    //Disable unneeded ints
    spi->hw->slave.rd_buf_done   		= 0;
    spi->hw->slave.wr_buf_done   		= 0;
    spi->hw->slave.rd_sta_done   		= 0;
    spi->hw->slave.wr_sta_done   		= 0;
    spi->hw->slave.rd_buf_inten  		= 0;
    spi->hw->slave.wr_buf_inten  		= 0;
    spi->hw->slave.rd_sta_inten  		= 0;
    spi->hw->slave.wr_sta_inten  		= 0;
    spi->hw->slave.trans_inten   		= 0;
    spi->hw->slave.trans_done    		= 0;

    //Configure clock
    s_myspi_configure_clock(spi);

    //Configure bit order
    spi->hw->ctrl.rd_bit_order           = 0;    // 0:MSB first. 1:LSB first
    spi->hw->ctrl.wr_bit_order           = 0;    // 0:MSB first. 1:LSB first

    spi->hw->user.wr_byte_order          = 1;    //BIG endian
    spi->hw->user.rd_byte_order          = 1;    //BIG endian

    //Configure polarity for mode 1 according to table 27 in tech data sheet p.125
    spi->hw->pin.ck_idle_edge            = 0;
    spi->hw->user.ck_out_edge            = 1;
    spi->hw->ctrl2.miso_delay_mode       = 0;
    spi->hw->ctrl2.miso_delay_num        = 0;
    spi->hw->ctrl2.mosi_delay_mode       = 0;
    spi->hw->ctrl2.mosi_delay_num        = 0;


    //configure dummy bits
    spi->hw->user.usr_dummy              = 0;
    spi->hw->user1.usr_dummy_cyclelen    = 0;

    //Set up QIO/DIO if needed
    spi->hw->ctrl.val		&= ~(SPI_FREAD_DUAL|SPI_FREAD_QUAD|SPI_FREAD_DIO|SPI_FREAD_QIO);
    spi->hw->user.val		&= ~(SPI_FWRITE_DUAL|SPI_FWRITE_QUAD|SPI_FWRITE_DIO|SPI_FWRITE_QIO);

    spi->hw->user1.usr_addr_bitlen       = 0;
    spi->hw->user2.usr_command_bitlen    = 0;
    spi->hw->user.usr_addr               = 0;
    spi->hw->user.usr_command            = 0;
    if(spi->dummy_cycle <= 0) {
        spi->hw->user.usr_dummy              = 0;
        spi->hw->user1.usr_dummy_cyclelen    = 0;
    } else {
        spi->hw->user.usr_dummy              = 1;
        spi->hw->user1.usr_dummy_cyclelen    = (uint8_t) (spi->dummy_cycle-1);
    }

    spi->hw->user.usr_mosi_highpart      = 0;
    spi->hw->user2.usr_command_value     = 0;

    //Configure MOSI/MISO
    spi->hw->user.usr_mosi               = 0;        
    spi->hw->user.usr_miso               = 0;
    spi->hw->mosi_dlen.usr_mosi_dbitlen  = 0;		// works great! (there's no glitch in 5 hours)
    spi->hw->miso_dlen.usr_miso_dbitlen  = 0;

    //Configure duplex mode
    spi->hw->user.doutdin                = 0;     //Full duplex disabled
    spi->hw->user.sio                    = 1;     //3-line half dublex enabled   

    //Configure address phase
    spi->hw->addr                        = 0;    //Address test. Will send MSB first if SPI_WR_BIT_ORDER = 0.
    spi->hw->user.usr_addr               = 0;   
    spi->hw->user1.usr_addr_bitlen       = 0;
    
    //Enable CS0
    spi->hw->pin.cs0_dis = 0;     

    //CS hold & setup time
    spi->hw->user.cs_hold = 1;
    spi->hw->ctrl2.hold_time = 3;
    spi->hw->user.cs_setup               = 1;
    spi->hw->ctrl2.setup_time            = 3;

    return ESP_OK;
}

esp_err_t myspi_DMA_init(spi_host_device_t spi_host, int dma_ch, uint32_t *buf)
{
    const bool dma_chan_claimed = spicommon_dma_chan_claim(spi_internal.dmaChan);
    if(! dma_chan_claimed) {
        spicommon_periph_free(spi_internal.host);
        return MY_ESP_ERR_SPI_DMA_ALREADY_IN_USE;
    }

    spi_dev_t* spi_hw = myspi_get_hw_for_host(spi_host);

    //Setup DMA descriptors
    spi_internal.descs = (lldesc_t *)calloc(2, sizeof(lldesc_t));

    ESP_LOGI(__func__, "Allocated memory for DMA desc. spi_internal.descs[0] address: %p", (void*) spi_internal.descs);

    spi_internal.descs[0].owner = 1;
    spi_internal.descs[0].eof = 1;
    spi_internal.descs[0].length = 2;
    spi_internal.descs[0].size = 4;     //Size must be word-aligned?
    spi_internal.descs[0].qe.stqe_next = spi_internal.descs+1;
    spi_internal.descs[0].buf = (uint8_t *) buf;

    spi_internal.descs[1].owner = 1;
    spi_internal.descs[1].eof = 1;
    spi_internal.descs[1].length = 2;
    spi_internal.descs[1].size = 4;
    spi_internal.descs[1].qe.stqe_next    = spi_internal.descs;
    spi_internal.descs[1].buf = (uint8_t *) (buf+1);

    //ESP_LOGI(__func__, "spi_internal.descs[0] address: %p", (void*) spi_internal.descs[0]);
    //ESP_LOGI(__func__, "spi_internal.descs[1] address: %p", (void*) spi_internal.descs[1]);

    ESP_LOGI(__func__, "DMA buffer 0 address: %p", (void*) spi_internal.descs[0].buf);
    ESP_LOGI(__func__, "DMA buffer 1 address: %p", (void*) spi_internal.descs[1].buf);

    //Select DMA channel
    DPORT_SET_PERI_REG_BITS(
          DPORT_SPI_DMA_CHAN_SEL_REG
        , 3
        , dma_ch
        , (spi_host* 2)
    );

    //Reset SPI DMA
    spi_hw->dma_conf.val        		|= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    spi_hw->dma_out_link.start  		= 0;
    spi_hw->dma_in_link.start   		= 0;
    spi_hw->dma_conf.val        		&= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    //Configure inlink descriptor
    spi_hw->dma_in_link.addr            = (int)(spi_internal.descs) & 0xFFFFF;
    spi_hw->dma_conf.indscr_burst_en    = 1;

    //Configure outlink descriptor
    spi_hw->dma_out_link.addr           = 0;
    spi_hw->dma_conf.out_data_burst_en  = 0;

    // Set circular mode
    //      https://www.esp32.com/viewtopic.php?f=2&t=4011#p18107
    //      > yes, in SPI DMA mode, SPI will alway transmit and receive
    //      > data when you set the SPI_DMA_CONTINUE(BIT16) of SPI_DMA_CONF_REG.
    //spi.hw->dma_conf.dma_tx_stop		= 1;	// Stop SPI DMA
    //spi.hw->dma_conf.dma_tx_stop		= 0;	// Disable stop
    //spi->hw->dma_conf.dma_continue       = 1;

    return ESP_OK;
}

esp_err_t myspi_init(my_spi_config_t *my_spi_config)
{
    esp_err_t ret;

    spi_internal.host = my_spi_config->host;
    spi_internal.dmaChan = my_spi_config->dmaChan;
    spi_internal.mosiGpioNum = my_spi_config->mosiGpioNum;
    spi_internal.sckGpioNum = my_spi_config->sckGpioNum;
    spi_internal.csGpioNum = my_spi_config->csGpioNum;
    spi_internal.clk_speed = my_spi_config->spi_clk;

    spi_internal.hw				= myspi_get_hw_for_host(spi_internal.host);
   
    //Configure SPI registers
    ret = s_myspi_configure_registers(&spi_internal);
    if (ret != ESP_OK) {
        ESP_LOGE(__func__, "s_myspi_configure_registers() returned %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(__func__, "SPI registers configured!");

    //Set up interrupt
    s_myspi_register_interrupt(&spi_internal);

    //Enable interrupt
    esp_intr_enable(spi_internal.intr);

    return ESP_OK;
}

esp_err_t myspi_deinit(my_spi_config_t *my_spi_config)
{
	spi_internal.hw = myspi_get_hw_for_host(my_spi_config->host);

	spi_internal.hw->dma_conf.dma_continue	= 0;
	spi_internal.hw->dma_out_link.start		= 0;
	spi_internal.hw->cmd.usr					= 0;

	// TODO : Reset GPIO Matrix

	spicommon_dma_chan_free(spi_internal.dmaChan);
	spicommon_periph_free(spi_internal.host);
	return ESP_OK;
}

esp_err_t myspi_start_transfers(void)
{
    //Todo: add init guard

    //spi_internal.hw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)
    spi_internal.hw->dma_in_link.start          = 1;
    spi_internal.hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer
    return ESP_OK;
}

esp_err_t myspi_set_addr(uint32_t addr, uint32_t len, bool enable)
{ 
    //Todo: add init guard
    if(enable)
    {
        //Configure address phase
        spi_internal.hw->addr          = (addr << len);    //Address test. Will send MSB first if SPI_WR_BIT_ORDER = 0.
        spi_internal.hw->user.usr_addr = 1;     //Enable address phase
        spi_internal.hw->user1.usr_addr_bitlen = len - 1;
    }
    else
    {
        
        spi_internal.hw->addr                        = 0x0;    //Address test. Will send MSB first if SPI_WR_BIT_ORDER = 0.
        spi_internal.hw->user.usr_addr = 0;   //Enable address phase
        spi_internal.hw->user1.usr_addr_bitlen = 0;   
    }
    return ESP_OK;
}

esp_err_t myspi_set_mosi(uint32_t len, bool enable)
{
    if(enable)
    {
        //Configure MOSI
        spi_internal.hw->user.usr_mosi               = 1;        
        spi_internal.hw->mosi_dlen.usr_mosi_dbitlen  = len-1;
    }
    else
    {
        spi_internal.hw->user.usr_mosi               = 0;        
        spi_internal.hw->mosi_dlen.usr_mosi_dbitlen  = 0;
    }
    
    return ESP_OK;
}

esp_err_t myspi_set_miso(uint32_t len, bool enable)
{
    if(enable)
    {
        //Configure MISO
        spi_internal.hw->user.usr_miso               = 1;        
        spi_internal.hw->miso_dlen.usr_miso_dbitlen  = len-1;
    }
    else
    {
        //Configure MOSI
        spi_internal.hw->user.usr_miso               = 0;        
        spi_internal.hw->miso_dlen.usr_miso_dbitlen  = 0;
    }
    
    return ESP_OK;
}

esp_err_t myspi_get_dma_data_rx(uint8_t *rxdata, uint32_t len_bytes)
{

    //Check register SPI_IN_SUC_EOF_DES_ADDR_REG to get "The last inlink descriptor address when SPI DMA encountered EOF. (RO)"
    //This way, we should be able to get the address of the buffer containing the most recent data.

    uint32_t dma_in_eof_addr_internal;
    uint32_t dma_data_size;
    uint8_t *dma_data_buf;

    //Get the last inlink descriptor address when SPI DMA encountered EOF
    //Todo - make atomic atomic_load()
    dma_in_eof_addr_internal = spi_internal.hw->dma_in_suc_eof_des_addr;
    
    //Assign pointer to the address
    spi_internal.last_inlink_desc_eof = (lldesc_t *)dma_in_eof_addr_internal;
    
    //Get the corresponding data buffer pointer
    dma_data_buf = spi_internal.last_inlink_desc_eof->buf;
    dma_data_size = spi_internal.last_inlink_desc_eof->length;

    //argument len is not really needed? Since we have the dma_data_size?
    //We could compare them as sanity check..

    for (size_t i = 0; i < dma_data_size; i++)
    {
        rxdata[i] = *dma_data_buf;
    }
    
    return ESP_OK;
}