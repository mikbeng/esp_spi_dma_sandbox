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

#define MSPI_DEBUG 1 

/* ========================================================================= */
/* [TYPE] Type definitions                                                   */
/* ========================================================================= */

/* ========================================================================= */
/* [PFDE] Private functions declaration                                      */
/* ========================================================================= */

/* ========================================================================= */
/* [GLOB] Global variables                                                   */
/* ========================================================================= */
static spi_internal_t spi_internal[3] = { 0 };

//Debug variables
volatile int int_cnt = 0;
volatile uint16_t test1, test2;

static const char * TAG = "mspi";
	
/* ========================================================================= */
/* [PFUN] Private functions implementations                                  */
/* ========================================================================= */

// This is run in interrupt context.
static void IRAM_ATTR s_spi_dma_intr(void *arg)
{
    spi_internal_t *spi_internal_p = (spi_internal_t *)arg;

    //Temporarily disable interrupt 
    esp_intr_disable(spi_internal_p->dma_intr);

    uint32_t spi_intr_status;
    uint32_t spi_intr_raw;

    spi_intr_raw = spi_internal_p->hw->dma_int_raw.val;    //Read raw interrupt bits
    spi_intr_status = spi_internal_p->hw->dma_int_st.val;  //Read interrupt status bits

    //Debug prints
    //ets_printf("DMA INT! spi_intr_status:%d\n", spi_intr_status);
    //ets_printf("DMA INT! spi_intr_raw:%d\n", spi_intr_raw);
    
    if (spi_intr_status & out_eof_int_en) { 

    }
    else if(spi_intr_status & in_suc_eof_int_en) { 
        spi_internal_p->hw->dma_in_link.start          = 1;
    }

    spi_internal_p->hw->dma_int_clr.val = spi_intr_status;     //Clears the interrupt

    //Finally, enable the interrupt
    esp_intr_enable(spi_internal_p->dma_intr);
}

// This is run in interrupt context.
static void IRAM_ATTR s_spi_trans_intr(void *arg)
{
    spi_internal_t *spi_internal_p = (spi_internal_t *)arg;
    //Temporarily disable interrupt 
    esp_intr_disable(spi_internal_p->trans_intr);

    uint32_t spi_intr_slave_val = 0;

    spi_intr_slave_val = spi_internal_p->hw->slave.val; //Read interrupt status

    //Debug prints
    //ets_printf("SPI TRANS INT! spi_intr_slave_val:%d\n", spi_intr_slave_val);

    if (spi_intr_slave_val & spi_trans_done_int) { 

        //Start new transfer if continuous mode active
        if(spi_internal_p->transfer_cont){
            spi_internal_p->hw->dma_in_link.start = 1;
            spi_internal_p->hw->cmd.usr = 1;	            // SPI: Start new SPI transfer
        }
        else{
            spi_internal_p->transfer_done = 1;
        }

        spi_internal_p->hw->slave.trans_done = 0;      //Clears the interrupt
    }
    
    //Finally, enable the interrupt
    esp_intr_enable(spi_internal_p->trans_intr);
}

static spi_dev_t *s_mspi_get_hw_for_host(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return &SPI1; break;
    case HSPI_HOST: return &SPI2; break;
    case VSPI_HOST: return &SPI3; break;
    default:        return NULL;  break;
    }
}


static esp_err_t s_getSpidOutByHost(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_OUT_IDX;	break;
    case HSPI_HOST: return HSPID_OUT_IDX;	break;
    case VSPI_HOST: return VSPID_OUT_IDX;	break;
    default:        return ESP_FAIL;	break;
    }
}


static esp_err_t s_getSpidInByHost(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_IN_IDX;		break;
    case HSPI_HOST: return HSPID_IN_IDX;	break;
    case VSPI_HOST: return VSPID_IN_IDX;	break;
    default:        return ESP_FAIL;	break;
    }
}

static esp_err_t s_mspi_register_interrupt(spi_internal_t *spi)
{
    esp_err_t ret;
    int spi_int_source, dma_int_source;

    switch (spi->host)
    {
    case HSPI_HOST: ///< SPI2, HSPI
        spi_int_source = ETS_SPI2_INTR_SOURCE;
        dma_int_source = ETS_SPI2_DMA_INTR_SOURCE;
        break;
    
    case VSPI_HOST: ///< SPI3, VSPI
        spi_int_source = ETS_SPI3_INTR_SOURCE;
        dma_int_source = ETS_SPI3_DMA_INTR_SOURCE;
    default:
        spi_int_source = ETS_SPI2_INTR_SOURCE;
        dma_int_source = ETS_SPI2_DMA_INTR_SOURCE;
        break;
    }

    int flags;
    int int_cpu;

    //DMA interrupt
    /*
    spi->hw->dma_int_ena.in_suc_eof = 1;
    flags = ESP_INTR_FLAG_IRAM;
    ret = esp_intr_alloc(dma_int_source, flags, &s_spi_dma_intr, (void*)&spi_internal[spi->host], &spi->dma_intr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_intr_alloc() returned %d", ret);
    }
    //esp_intr_enable(spi.trans_intr);
    int_cpu = esp_intr_get_cpu(spi->dma_intr);
    ESP_LOGI(TAG, "Allocated interrupt for hardware source: %d on cpu %d", dma_int_source, int_cpu);
    */

    //SPI interrupt

    spi->hw->slave.trans_done = 0;      //Clear any pending interrupt

    spi->hw->slave.trans_inten = 1;
    flags = ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED;
    ret = esp_intr_alloc(spi_int_source, flags, &s_spi_trans_intr, (void*)&spi_internal[spi->host], &spi->trans_intr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_intr_alloc() returned %d", ret);
        return ESP_FAIL;
    }
    int_cpu = esp_intr_get_cpu(spi->trans_intr);
    ESP_LOGI(TAG, "Allocated interrupt for hardware source: %d on cpu %d", spi_int_source, int_cpu);
    return ESP_OK;
}

static esp_err_t s_mspi_configure_clock(spi_internal_t *spi)
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

static esp_err_t s_mspi_configure_GPIO(spi_internal_t *spi)
{
    //Configure GPIOs for 3-wire half duplex (MOSI,SCK and CS)
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[spi->mosiGpioNum], PIN_FUNC_GPIO);
    gpio_set_direction(spi->mosiGpioNum, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(spi->mosiGpioNum, s_getSpidOutByHost(spi->host), false, false);
    gpio_matrix_in(spi->mosiGpioNum, s_getSpidInByHost(spi->host), false);

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

static esp_err_t s_mspi_configure_registers(spi_internal_t *spi) 
{

    //Claim peripheral
    const bool spi_periph_claimed = spicommon_periph_claim(spi->host);
    if(! spi_periph_claimed) {
        return MY_ESP_ERR_SPI_HOST_ALREADY_IN_USE;
    }

    s_mspi_configure_GPIO(spi);

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
    s_mspi_configure_clock(spi);

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
    spi->hw->mosi_dlen.usr_mosi_dbitlen  = 0;
    spi->hw->miso_dlen.usr_miso_dbitlen  = 0;

    //Configure duplex mode
    spi->hw->user.doutdin                = 0;     //Full duplex disabled
    spi->hw->user.sio                    = 1;     //3-line half dublex enabled   

    //Configure address phase
    spi->hw->addr                        = 0;
    spi->hw->user.usr_addr               = 0;
    spi->hw->user1.usr_addr_bitlen       = 0;
    
    //Enable CS0
    spi->hw->pin.cs0_dis = 0;     

    //CS hold & setup time
    spi->hw->user.cs_hold               = 1;
    spi->hw->ctrl2.hold_time            = 3;
    spi->hw->user.cs_setup              = 1;
    spi->hw->ctrl2.setup_time           = 3;

    return ESP_OK;
}

esp_err_t mspi_DMA_init(mspi_dma_config_t *mspi_dma_config, mspi_device_handle_t handle)
{
    if(mspi_dma_config->dmaChan == 0)
    {
        ESP_LOGI(TAG, "DMA channel = 0. No DMA will be used");
        return ESP_OK;
    }

    const bool dma_chan_claimed = spicommon_dma_chan_claim(mspi_dma_config->dmaChan);
    if(! dma_chan_claimed) {
        return MY_ESP_ERR_SPI_DMA_ALREADY_IN_USE;
    }

    handle->dmaChan = mspi_dma_config->dmaChan;

    spi_dev_t* spi_hw = s_mspi_get_hw_for_host(handle->host);

    //Set up DMA buffer
    //Total length of buffer is: number of buffers * size of each buffer
    uint32_t len_bytes = sizeof(uint32_t) * mspi_dma_config->list_buf_size * mspi_dma_config->list_num;

    ESP_LOGD(TAG, "Allocating DMA buffer with %d bytes", len_bytes);
    handle->dma_buffer = (uint32_t *)heap_caps_malloc(len_bytes, MALLOC_CAP_DMA);       	//For DMA

    ESP_LOGD(TAG, "Successfully allocated spi_buffer on address: %p",handle->dma_buffer);

    //Setup DMA descriptors
    handle->descs = (lldesc_t *)calloc(mspi_dma_config->list_num, sizeof(lldesc_t));

    //See the spicommon_setup_dma_desc_links() function in spi_common.c for insperation

    for (size_t i = 0; i < mspi_dma_config->list_num; i++)
    {
        handle->descs[i].owner = 1;
        handle->descs[i].eof = 1;      //Hard-coded to 1 for now. This makes the SPI_IN_SUC_EOF_DES_ADDR_REG updated at each dma transfer
        handle->descs[i].length = mspi_dma_config->dma_trans_len;   
        handle->descs[i].size = mspi_dma_config->list_buf_size;     
        handle->descs[i].qe.stqe_next = handle->descs+1;
        handle->descs[i].buf = (uint8_t *) handle->dma_buffer;
    }

    //Configure inlink descriptor
    spi_hw->dma_in_link.addr            = (int)(handle->descs) & 0xFFFFF;
    spi_hw->dma_conf.indscr_burst_en    = 1;

    //ESP_LOGI(TAG, "DMA buffer 0 address: %p", (void*) spi_internal.descs[0].buf);
    //ESP_LOGI(TAG, "DMA buffer 1 address: %p", (void*) spi_internal.descs[1].buf);

    //Select DMA channel
    DPORT_SET_PERI_REG_BITS(
          DPORT_SPI_DMA_CHAN_SEL_REG
        , 3
        , mspi_dma_config->dmaChan
        , (handle->host* 2)
    );

    //Reset SPI DMA
    spi_hw->dma_conf.val        		|= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    spi_hw->dma_out_link.start  		= 0;
    spi_hw->dma_in_link.start   		= 0;
    spi_hw->dma_conf.val        		&= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    //Configure outlink descriptor
    spi_hw->dma_out_link.addr           = 0;
    spi_hw->dma_conf.out_data_burst_en  = 0;

    return ESP_OK;
}
esp_err_t mspi_DMA_deinit(mspi_device_handle_t handle)
{
    free(handle->descs);
    free(handle->dma_buffer);

    spicommon_dma_chan_free(handle->dmaChan);

    return ESP_OK;
}

esp_err_t mspi_init(mspi_config_t *mspi_config, mspi_device_handle_t* handle)
{
    esp_err_t ret;
    spi_host_device_t host = mspi_config->host;

    spi_internal[host].host = mspi_config->host;
    spi_internal[host].mosiGpioNum = mspi_config->mosiGpioNum;
    spi_internal[host].sckGpioNum = mspi_config->sckGpioNum;
    spi_internal[host].csGpioNum = mspi_config->csGpioNum;
    spi_internal[host].clk_speed = mspi_config->spi_clk;

    spi_internal[host].hw				= s_mspi_get_hw_for_host(spi_internal[host].host);
    
    //Configure SPI registers
    ret = s_mspi_configure_registers(&spi_internal[host]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "s_mspi_configure_registers() returned %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SPI registers configured!");

    //Set up interrupt
    s_mspi_register_interrupt(&spi_internal[host]);

    spi_internal[host].initiated = true;
    *handle = &spi_internal[host];

    return ESP_OK;
}

esp_err_t mspi_deinit(mspi_device_handle_t handle)
{
	handle->hw->dma_conf.dma_continue	= 0;
	handle->hw->dma_out_link.start		= 0;
	handle->hw->cmd.usr					= 0;

    esp_intr_disable(handle->trans_intr);
    esp_intr_free(handle->trans_intr);

	// TODO : Reset GPIO Matrix
	spicommon_periph_free(handle->host);
	return ESP_OK;
}

esp_err_t mspi_start_continuous_DMA_rx(mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(handle->dmaChan == 0){
        ESP_LOGE(TAG, "DMA not used! Init DMA first with channel 1 or 2");
        return ESP_FAIL;
    }
    
    handle->transfer_cont               = 1;
    handle->hw->dma_in_link.start       = 1;
    handle->hw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer

    handle->hw->slave.trans_done = 0;      //Clear any pending interrupt
    esp_intr_enable(handle->trans_intr);

    return ESP_OK;
}

esp_err_t mspi_stop_continuous_DMA_rx(mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(handle->dmaChan == 0){
        ESP_LOGE(TAG, "DMA not used! Init DMA first with channel 1 or 2");
        return ESP_FAIL;
    }

    if(handle->transfer_cont == 0){
        ESP_LOGW(TAG, "mspi_stop_continuous_DMA_rx: Continuous transfers not active. Nothing to be done.");
        return ESP_OK;
    }
    
    esp_intr_disable(handle->trans_intr);

    handle->transfer_cont               = 0;
    handle->hw->dma_in_link.stop        = 1;
    handle->hw->dma_in_link.restart     = 1;

    return ESP_OK;
}

esp_err_t mspi_device_poll(uint32_t addr, uint32_t len, bool enable, mspi_device_handle_t handle)
{ 
    
    return ESP_OK;
}

esp_err_t mspi_set_addr(uint32_t addr, uint32_t len, bool enable, mspi_device_handle_t handle)
{ 
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }

    if(enable)
    {
        //Configure address phase
        handle->hw->addr          = (addr << len);    //Address test. Will send MSB first if SPI_WR_BIT_ORDER = 0.
        handle->hw->user.usr_addr = 1;     //Enable address phase
        handle->hw->user1.usr_addr_bitlen = len - 1;
    }
    else
    {
        
        handle->hw->addr                        = 0x0;    //Address test. Will send MSB first if SPI_WR_BIT_ORDER = 0.
        handle->hw->user.usr_addr = 0;   //Enable address phase
        handle->hw->user1.usr_addr_bitlen = 0;   
    }
    return ESP_OK;
}

esp_err_t mspi_set_mosi(uint32_t len, bool enable, mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(enable)
    {
        //Configure MOSI
        handle->hw->user.usr_mosi               = 1;        
        handle->hw->mosi_dlen.usr_mosi_dbitlen  = len-1;
    }
    else
    {
        handle->hw->user.usr_mosi               = 0;        
        handle->hw->mosi_dlen.usr_mosi_dbitlen  = 0;
    }
    
    return ESP_OK;
}

esp_err_t mspi_set_miso(uint32_t len, bool enable, mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(enable)
    {
        //Configure MISO
        handle->hw->user.usr_miso               = 1;        
        handle->hw->miso_dlen.usr_miso_dbitlen  = len-1;
    }
    else
    {
        //Configure MOSI
        handle->hw->user.usr_miso               = 0;        
        handle->hw->miso_dlen.usr_miso_dbitlen  = 0;
    }
    
    return ESP_OK;
}

esp_err_t mspi_get_dma_data_rx(uint8_t *rxdata, uint32_t *rx_len_bytes, mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(handle->dmaChan == 0){
        ESP_LOGE(TAG, "DMA not used! Init DMA first with channel 1 or 2");
        return ESP_FAIL;
    }
    //Check register SPI_IN_SUC_EOF_DES_ADDR_REG to get "The last inlink descriptor address when SPI DMA encountered EOF. (RO)"
    //This way, we should be able to get the address of the buffer containing the most recent data.

    //TODO - Check so that we actually have some data received!

    uint32_t dma_in_eof_addr_internal;
    lldesc_t* last_inlink_desc_eof;
    uint32_t dma_data_size;
    uint8_t *dma_data_buf;

    //Get the last inlink descriptor address when SPI DMA encountered EOF
    dma_in_eof_addr_internal = handle->hw->dma_in_suc_eof_des_addr;
    
    //Assign pointer to the address
    last_inlink_desc_eof = (lldesc_t *)dma_in_eof_addr_internal;
    
    //Get the corresponding data buffer pointer
    dma_data_buf = last_inlink_desc_eof->buf;
    dma_data_size = last_inlink_desc_eof->length;

    *rx_len_bytes = dma_data_size;

    for (size_t i = 0; i < dma_data_size; i++)
    {
        rxdata[i] = *(dma_data_buf+i);
    }
    
    return ESP_OK;
}