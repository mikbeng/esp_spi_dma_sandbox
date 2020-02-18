/* ========================================================================= */
/* [INCL] Includes                                                           */
/* ========================================================================= */
#include <soc/spi_reg.h>
#include "soc/spi_struct.h"
#include <soc/dport_reg.h>
#include "driver/periph_ctrl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "my_spi.h"
#include "esp_log.h"
#include "string.h"



/* ========================================================================= */
/* [DEFS] Defines                                                            */
/* ========================================================================= */
#define in_suc_eof_int_en BIT(5)
#define SPI_IN_ERR_EOF_INT BIT(4)
#define SPI_INLINK_DSCR_ERROR_INT BIT(2)
#define SPI_INLINK_DSCR_EMPTY_INT BIT(0)

#define out_eof_int_en BIT(7)
#define spi_trans_done_int BIT(4)

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
int level = 0;

static const char * TAG = "mspi";
	
/* ========================================================================= */
/* [PFUN] Private functions implementations                                  */
/* ========================================================================= */

// This is run in interrupt context.
static void IRAM_ATTR s_spi_dma_intr(void *arg)
{
    spi_internal_t *spi_internal_p = (spi_internal_t *)arg;
    static volatile int cnt = 0;
    //Temporarily disable interrupt 
    esp_intr_disable(spi_internal_p->dma_handle.dma_intr);

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

        //spi_internal_p->hw->dma_in_link.start          = 1;
    }

    spi_internal_p->hw->dma_int_clr.val = spi_intr_status;     //Clears the interrupt

    //Finally, enable the interrupt
    esp_intr_enable(spi_internal_p->dma_handle.dma_intr);
}

// This is run in interrupt context.
static void IRAM_ATTR s_spi_trans_intr(void *arg)
{
    spi_internal_t *spi_internal_p = (spi_internal_t *)arg;

    //Temporarily disable interrupt 
    esp_intr_disable(spi_internal_p->trans_intr);

    uint32_t spi_intr_slave_val = spi_internal_p->hw->slave.val; //Read interrupt status

    //Debug prints
    //ets_printf("SPI TRANS INT! spi_intr_slave_val:%d\n", spi_intr_slave_val);

    if (spi_intr_slave_val & spi_trans_done_int) { 

        // //Start new transfer if continuous mode active
        // if(spi_internal_p->transfer_cont){

        //     //Check if it's rx or tx transfers (MISO or MOSI)
        //     if(spi_internal_p->hw->user.usr_mosi == 1){
        //         spi_internal_p->hw->dma_out_link.start = 1;
        //     }
        //     if(spi_internal_p->hw->user.usr_miso == 1){
        //         spi_internal_p->hw->dma_in_link.start = 1;
        //     }
            
        //     spi_internal_p->hw->cmd.usr = 1;	            // SPI: Start new SPI transfer
        // }
        // else{
        //     spi_internal_p->polling_done = 1;
        // }

        if(spi_internal_p->polling_active){
            spi_internal_p->polling_done = 1;
        }

        spi_internal_p->hw->slave.trans_done = 0;      //Clears the interrupt
    }

    //GPIO debug pin
    //level ^= 1;
    //gpio_set_level(GPIO_NUM_2, level);

    //Finally, enable the interrupt
    //esp_intr_enable(spi_internal_p->trans_intr);
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

static esp_err_t s_mspi_get_spid_out_for_host(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_OUT_IDX;	break;
    case HSPI_HOST: return HSPID_OUT_IDX;	break;
    case VSPI_HOST: return VSPID_OUT_IDX;	break;
    default:        return ESP_FAIL;	break;
    }
}


static esp_err_t s_mspi_get_spid_in_for_host(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_IN_IDX;		break;
    case HSPI_HOST: return HSPID_IN_IDX;	break;
    case VSPI_HOST: return VSPID_IN_IDX;	break;
    default:        return ESP_FAIL;	break;
    }
}

static esp_err_t s_mspi_register_interrupt_dmatrans(spi_internal_t *spi)
{
    esp_err_t ret;
    int dma_int_source;

    switch (spi->host)
    {
    case HSPI_HOST: ///< SPI2, HSPI
        dma_int_source = ETS_SPI2_DMA_INTR_SOURCE;
        break;
    
    case VSPI_HOST: ///< SPI3, VSPI
        dma_int_source = ETS_SPI3_DMA_INTR_SOURCE;
    default:
        dma_int_source = ETS_SPI2_DMA_INTR_SOURCE;
        break;
    }

    //Clear any pending interrupt
    spi->hw->dma_int_clr.val = 0x000001FF;  //Clears all pending interrupts

    //Register DMA interrupts
    spi->hw->dma_int_ena.in_suc_eof = 1;     //Enable SPI_IN_SUC_EOF_INT

    int flags = ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED;
    ret = esp_intr_alloc(dma_int_source, flags, &s_spi_dma_intr, (void*)&spi_internal[spi->host], &spi->dma_handle.dma_intr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_intr_alloc() returned %d", ret);
    }
    //esp_intr_enable(spi.trans_intr);
    int int_cpu = esp_intr_get_cpu(spi->dma_handle.dma_intr);
    ESP_LOGI(TAG, "Allocated interrupt for hardware source: %d on cpu %d", dma_int_source, int_cpu);

    return ESP_OK;
}

static esp_err_t s_mspi_register_interrupt_spitrans(spi_internal_t *spi)
{
    esp_err_t ret;
    int spi_int_source;

    switch (spi->host)
    {
    case HSPI_HOST: ///< SPI2, HSPI
        spi_int_source = ETS_SPI2_INTR_SOURCE;
        break;
    
    case VSPI_HOST: ///< SPI3, VSPI
        spi_int_source = ETS_SPI3_INTR_SOURCE;
    default:
        spi_int_source = ETS_SPI2_INTR_SOURCE;
        break;
    }

    //SPI interrupt
    spi->hw->slave.trans_done = 0;      //Clear any pending interrupt

    spi->hw->slave.trans_inten = 1;
    int flags = ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED;
    ret = esp_intr_alloc(spi_int_source, flags, &s_spi_trans_intr, (void*)&spi_internal[spi->host], &spi->trans_intr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_intr_alloc() returned %d", ret);
        return ESP_FAIL;
    }
    int int_cpu = esp_intr_get_cpu(spi->trans_intr);
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
    gpio_matrix_out(spi->mosiGpioNum, s_mspi_get_spid_out_for_host(spi->host), false, false);
    gpio_matrix_in(spi->mosiGpioNum, s_mspi_get_spid_in_for_host(spi->host), false);

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
        ESP_LOGE(TAG, "SPI periph not claimed. Already in use.");
        return ESP_FAIL;   
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
    spi->hw->pin.ck_idle_edge            = 0;   //CPOL = 0
    spi->hw->user.ck_out_edge            = 1;   //CPHA = 1
    spi->hw->ctrl2.miso_delay_mode       = 0;   
    spi->hw->ctrl2.miso_delay_num        = 0;
    spi->hw->ctrl2.mosi_delay_mode       = 0;
    spi->hw->ctrl2.mosi_delay_num        = 0;


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
        spi->hw->user.usr_dummy_idle         = 0;
    } else {
        spi->hw->user.usr_dummy              = 1;                               //This bit enables the dummy phase of an SPI operation in SPI half-duplex mode
        spi->hw->user1.usr_dummy_cyclelen    = (uint8_t) (spi->dummy_cycle-1);  //The number of SPI clock cycles for the dummy phase minus one in SPI half-duplex mode
        spi->hw->user.usr_dummy_idle         = 1;
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

static esp_err_t s_mspi_set_addr(uint32_t addr, uint32_t len, bool enable, spi_internal_t *spi)
{ 
    if(spi->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }

    if(enable)
    {
        //Configure address phase
        spi->hw->addr          = (addr << len);    //Address, will send MSB first if SPI_WR_BIT_ORDER = 0.
        spi->hw->user.usr_addr = 1;                //Enable address phase
        spi->hw->user1.usr_addr_bitlen = len - 1;
    }
    else
    {
        
        spi->hw->addr                        = 0x0;    //Address, will send MSB first if SPI_WR_BIT_ORDER = 0.
        spi->hw->user.usr_addr = 0;   //Enable address phase
        spi->hw->user1.usr_addr_bitlen = 0;   
    }
    return ESP_OK;
}

static esp_err_t s_mspi_set_cmd(uint16_t cmd, uint32_t len, bool enable, spi_internal_t *spi)
{ 
    if(spi->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }

    uint16_t cmd_low, cmd_high;
    if(enable)
    {
        //Configure cmd phase
        cmd_high = (cmd >> 8);
        cmd_low = (cmd & 0x00FF);
        spi->hw->user2.usr_command_value  = (cmd_low << 8) | cmd_high;    /*The value of  command. Output sequence: bit 7-0 and then 15-8.*/
        spi->hw->user.usr_command         = 1;                           //Enable command phase
        spi->hw->user2.usr_command_bitlen = len - 1;
    }
    else
    {
        
        spi->hw->user2.usr_command_value  = 0;    /*The value of  command. Output sequence: bit 7-0 and then 15-8.*/
        spi->hw->user.usr_command         = 0;                //Disable command phase
        spi->hw->user2.usr_command_bitlen = 0;  
    }
    return ESP_OK;
}

static esp_err_t s_mspi_set_mosi(uint32_t len, bool enable, spi_internal_t *spi)
{
    if(spi->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(enable)
    {
        //Configure MOSI
        spi->hw->user.usr_mosi               = 1;        
        spi->hw->mosi_dlen.usr_mosi_dbitlen  = len-1;
    }
    else
    {
        spi->hw->user.usr_mosi               = 0;        
        spi->hw->mosi_dlen.usr_mosi_dbitlen  = 0;
    }
    
    return ESP_OK;
}

static esp_err_t s_mspi_set_miso(uint32_t len, bool enable, spi_internal_t *spi)
{
    if(spi->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(enable)
    {
        //Configure MISO
        spi->hw->user.usr_miso               = 1;        
        spi->hw->miso_dlen.usr_miso_dbitlen  = len-1;
    }
    else
    {
        //Configure MOSI
        spi->hw->user.usr_miso               = 0;        
        spi->hw->miso_dlen.usr_miso_dbitlen  = 0;
    }
    
    return ESP_OK;
}

/* ========================================================================= */
/* [FUNC] Functions implementations                                          */
/* ========================================================================= */

esp_err_t mspi_DMA_init(mspi_dma_config_t *mspi_dma_config, mspi_device_handle_t handle)
{
    if(mspi_dma_config->dmaChan == 0)
    {
        ESP_LOGI(TAG, "DMA channel = 0. No DMA will be used");
        return ESP_OK;
    }


    //Claim the DMA Peripheral
    const bool dma_chan_claimed = spicommon_dma_chan_claim(mspi_dma_config->dmaChan);
    if(! dma_chan_claimed) {
        ESP_LOGE(TAG, "DMA periph not claimed. Already in use.");
        return ESP_FAIL; 
    }

    //Select DMA channel
    DPORT_SET_PERI_REG_BITS(
          DPORT_SPI_DMA_CHAN_SEL_REG
        , 3
        , mspi_dma_config->dmaChan
        , (handle->host* 2)
    );

    handle->dma_handle.dmaChan = mspi_dma_config->dmaChan;

    //Calculate the DMA transfer length
    uint32_t dmachunklen;
    if (mspi_dma_config->isrx) {
        //Receive needs DMA length rounded to next 32-bit boundary
        dmachunklen = (mspi_dma_config->dma_trans_len + 3) & (~3);
    } else {
        dmachunklen = mspi_dma_config->dma_trans_len;
    }    

    //Set up DMA buffer
    handle->dma_handle.buffer_len = dmachunklen * mspi_dma_config->list_num;

    ESP_LOGD(TAG, "Allocating DMA buffer with %d bytes", handle->dma_handle.buffer_len);
    handle->dma_handle.dma_buffer = (uint32_t *)heap_caps_malloc(handle->dma_handle.buffer_len, MALLOC_CAP_DMA);       	//For DMA
    
    memset(handle->dma_handle.dma_buffer, 0, handle->dma_handle.buffer_len);
    ESP_LOGD(TAG, "Successfully allocated spi_buffer on address: %p", handle->dma_handle.dma_buffer);
 

    //Setup DMA descriptors
    handle->dma_handle.descs = (lldesc_t *)calloc(mspi_dma_config->list_num, sizeof(lldesc_t));
    uint8_t *data = (uint8_t *)handle->dma_handle.dma_buffer;

    for (size_t i = 0; i < mspi_dma_config->list_num; i++)
    {
        handle->dma_handle.descs[i].owner = 1;
        handle->dma_handle.descs[i].eof = 1;      //Hard-coded to 1 for now. This makes the SPI_IN_SUC_EOF_DES_ADDR_REG updated at each dma transfer
        handle->dma_handle.descs[i].sosf = 0; 

        handle->dma_handle.descs[i].length = 2;
        handle->dma_handle.descs[i].size = dmachunklen;
        
        handle->dma_handle.descs[i].qe.stqe_next = &handle->dma_handle.descs[i+1];
        handle->dma_handle.descs[i].buf = data;

        ESP_LOGD(TAG, "DMA desc %d address: %p", i,(void*) &handle->dma_handle.descs[i]);
        ESP_LOGD(TAG, "DMA buffer %d address: %p", i,(void*) handle->dma_handle.descs[i].buf);
       
        data += dmachunklen;    //Increment buffer pointer
    }

    handle->dma_handle.descs[mspi_dma_config->list_num - 1].eof = 1;             //Mark last DMA desc as end of stream.

    if(mspi_dma_config->linked_list_circular)
    {
        handle->dma_handle.descs[mspi_dma_config->list_num - 1].qe.stqe_next = &handle->dma_handle.descs[0]; //Point to the first descriptor to get a circular array of descriptors
    }
    else
    {
        handle->dma_handle.descs[mspi_dma_config->list_num - 1].qe.stqe_next = NULL; //current linked list item is last of list
    }
    
    //Reset SPI DMA
    handle->hw->dma_conf.val        		|= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    handle->hw->dma_out_link.start  		= 0;
    handle->hw->dma_in_link.start   		= 0;
    handle->hw->dma_conf.val        		&= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    if(mspi_dma_config->isrx == 0){
        //Configure outlink descriptor
        handle->hw->dma_out_link.addr           = (int)(handle->dma_handle.descs) & 0xFFFFF;
        handle->hw->dma_conf.outdscr_burst_en   = 1;
        handle->hw->dma_conf.out_data_burst_en  = 0;
        handle->hw->dma_out_link.start		    = 1;	
    }
    else
    {
        //Configure inlink descriptor
        handle->hw->dma_in_link.addr            = (int)(handle->dma_handle.descs) & 0xFFFFF; 
        handle->hw->dma_conf.indscr_burst_en    = 1;
        handle->hw->dma_in_link.start           = 1;
    }

    //Register any DMA interrupts if needed
    //s_mspi_register_interrupt_dmatrans(handle);

    return ESP_OK;
}
esp_err_t mspi_DMA_deinit(mspi_device_handle_t handle)
{
    free(handle->dma_handle.descs);
    free(handle->dma_handle.dma_buffer);

    spicommon_dma_chan_free(handle->dma_handle.dmaChan);

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

    spi_internal[host].dummy_cycle = mspi_config->dummy_cycle;

    spi_internal[host].hw				= s_mspi_get_hw_for_host(spi_internal[host].host);
    
    //Configure SPI registers
    ret = s_mspi_configure_registers(&spi_internal[host]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "s_mspi_configure_registers() returned %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SPI registers configured!");

    //Set up interrupt
    s_mspi_register_interrupt_spitrans(&spi_internal[host]);

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

esp_err_t mspi_start_continuous_DMA(mspi_transaction_t *mspi_trans_p, mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(handle->dma_handle.dmaChan == 0){
        ESP_LOGE(TAG, "DMA not used! Init DMA first with channel 1 or 2");
        return ESP_FAIL;
    }
    
    //Reset miso,mosi and address
    s_mspi_set_mosi(0,0, handle);
    s_mspi_set_miso(0,0, handle);
    s_mspi_set_addr(0,0,0, handle);

    if(mspi_trans_p->addr_len != 0){
        s_mspi_set_addr(mspi_trans_p->addr, mspi_trans_p->addr_len, 1, handle);
    }
    if(mspi_trans_p->cmd_len != 0){
        s_mspi_set_cmd(mspi_trans_p->cmd, mspi_trans_p->cmd_len, 1, handle);
    }

    if((mspi_trans_p->tx_len != 0) && (mspi_trans_p->rx_len != 0)){
        ESP_LOGE(TAG, "DMA rx and tx simultaniously not supported at the moment!");
        return ESP_FAIL;
    }
    
    if(mspi_trans_p->tx_len != 0){
        //Set up MOSI phase
        s_mspi_set_mosi(mspi_trans_p->tx_len, 1, handle);
        
        //Reset dma tx
        handle->hw->dma_conf.dma_tx_stop		    = 1;	// Stop SPI DMA
        handle->hw->dma_conf.dma_tx_stop		    = 0;	// Disable stop
        handle->hw->dma_out_link.start              = 1;
    }

    if(mspi_trans_p->rx_len != 0){
        //Set up MISO phase
        s_mspi_set_miso(mspi_trans_p->rx_len, 1, handle);
        handle->hw->dma_in_link.start               = 1;
        //Reset dma rx
        handle->hw->dma_conf.dma_rx_stop		    = 1;	// Stop SPI DMA
        handle->hw->dma_conf.dma_rx_stop		    = 0;	// Disable stop
    }

    //Kick off continuous transfers
    handle->hw->dma_conf.dma_continue		    = 1;
    handle->hw->cmd.usr                         = 1;

    //Set internal flag
    handle->transfer_cont               = 1;

    return ESP_OK;
}

esp_err_t mspi_stop_continuous_DMA(mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(handle->dma_handle.dmaChan == 0){
        ESP_LOGE(TAG, "DMA not used! Init DMA first with channel 1 or 2");
        return ESP_FAIL;
    }

    if(handle->transfer_cont == 0){
        ESP_LOGW(TAG, "mspi_stop_continuous_DMA_rx: Continuous transfers not active. Nothing to be done.");
        return ESP_OK;
    }
    
    //esp_intr_disable(handle->trans_intr);
    handle->hw->cmd.usr                 = 0;
    handle->hw->dma_conf.dma_rx_stop    = 1;	// Stop SPI DMA

    handle->hw->dma_in_link.stop        = 1;
    handle->hw->dma_out_link.stop       = 1;
    
    //Reset SPI DMA
    handle->hw->dma_conf.val        		|= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    handle->hw->dma_out_link.start  		= 0;
    handle->hw->dma_in_link.start   		= 0;
    handle->hw->dma_conf.val        		&= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    //reset buffer
    memset(handle->dma_handle.dma_buffer, 0, handle->dma_handle.buffer_len);

    // Reset SPI DMA periph
    periph_module_reset( PERIPH_SPI_DMA_MODULE );

    handle->transfer_cont               = 0;

    return ESP_OK;
}

esp_err_t mspi_device_transfer_blocking(mspi_transaction_t *mspi_trans_p, mspi_device_handle_t handle)
{ 
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }

    //Check if transfer_cont==1. If so, do not allow any polling since the device is busy.
    if(handle->transfer_cont == 1){
        ESP_LOGE(TAG, "Continuous transfers active! Polling not allowed.");
        return ESP_FAIL;  
    }

    handle->polling_done = 0;
    handle->polling_active = 1;

    handle->hw->slave.trans_done = 0;       //Clear any pending interrupt
    esp_intr_enable(handle->trans_intr);    //Enable spi interrupt
    handle->hw->cmd.usr = 1;	            // SPI: Start new SPI transfer

    while(handle->polling_done != 1){
        //vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Fetch the data from FIFO.
    if(mspi_trans_p->rx_len > 0)
    {
        if(mspi_trans_p->rxdata == NULL){
            ESP_LOGE(TAG, "rxdata buffer = NULL! Returning.");
            return ESP_FAIL;  
        }

        int word_len = (mspi_trans_p->rx_len + 31)/32;    //How many uint32 do we need for rx_len.
        int byte_len = (mspi_trans_p->rx_len + 7)/8;      //How many bytes do we need for rx_len.

        for (int i = 0; i < word_len; i++)
        {
            uint32_t word = handle->hw->data_buf[i];
            for (int j = 0; j < byte_len; j++)
            {
                mspi_trans_p->rxdata[j] = (uint8_t)((word >> (8*j)) & 0xFF);
            }  
        }
          
    }
    

    // // The function is called when a transaction is done, in ISR or in the task.
    // // Fetch the data from FIFO and call the ``post_cb``.
    // static void SPI_MASTER_ISR_ATTR spi_post_trans(spi_host_t *host)
    // {
    //     spi_transaction_t *cur_trans = host->cur_trans_buf.trans;
    //     if (host->cur_trans_buf.buffer_to_rcv && host->dma_chan == 0 ) {
    //         //Need to copy from SPI regs to result buffer.
    //         for (int x = 0; x < cur_trans->rxlength; x += 32) {
    //             //Do a memcpy to get around possible alignment issues in rx_buffer
    //             uint32_t word = host->hw->data_buf[x / 32];
    //             int len = cur_trans->rxlength - x;
    //             if (len > 32) len = 32;
    //             memcpy(&host->cur_trans_buf.buffer_to_rcv[x / 32], &word, (len + 7) / 8);
    //         }
    //     }
    //     //Call post-transaction callback, if any
    //     spi_device_t* dev = atomic_load(&host->device[host->cur_cs]);
    //     if (dev->cfg.post_cb) dev->cfg.post_cb(cur_trans);

    //     host->cur_cs = NO_CS;
    // }

    handle->polling_active = 0;

    return ESP_OK;
}

esp_err_t mspi_get_dma_data_rx(uint8_t *rxdata, uint32_t *rx_len_bytes, mspi_device_handle_t handle)
{
    if(handle->initiated == false){
        ESP_LOGE(TAG, "mspi not initiated! Init first");
        return ESP_FAIL;    
    }
    if(handle->dma_handle.dmaChan == 0){
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
    
    if(dma_in_eof_addr_internal == 0){
        //No EOF encountered yet.
        ESP_LOGW(TAG, "No DMA data ready yet");
        return ESP_OK;
    }

    //Assign pointer to the address
    last_inlink_desc_eof = (lldesc_t *)dma_in_eof_addr_internal;
    
    //Get the corresponding data buffer pointer
    dma_data_buf = (uint8_t *) last_inlink_desc_eof->buf;
    dma_data_size = last_inlink_desc_eof->length;

    ESP_LOGI(TAG, "last_inlink_desc_eof:%p", last_inlink_desc_eof);
    ESP_LOGI(TAG, "dma_data_buf address:%p. Value:[0x%02x, 0x%02x]", dma_data_buf, *dma_data_buf, *(dma_data_buf+1));
    ESP_LOGI(TAG, "dma_data_size:%d", dma_data_size);

    *rx_len_bytes = dma_data_size;

    for (size_t i = 0; i < dma_data_size; i++)
    {
        rxdata[i] = *(dma_data_buf+i);
    }
    
    return ESP_OK;
}