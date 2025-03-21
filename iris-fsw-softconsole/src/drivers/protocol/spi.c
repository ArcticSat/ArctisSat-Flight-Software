//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  SPI tasks and functions for SPI masters.
//
// History
// 2019-02-08 by Tamkin Rahman
// - Created.
// 2019-02-24 by Tamkin Rahman
// - Remove the use of mutex within spi.c functions. Instead, the user will have access to the mutexes via the header file.
// 2019-03-28 by Tamkin Rahman
// - Correct file description.
// 2019-04-17 by Tamkin Rahman
// - Allow the user to register a GPIO to use as the slave select to avoid toggling the slave select between byte transfers. Also,
//   add new functions to allow the user to use a GPIO for the slave select.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "drivers/protocol/spi.h"

#include <string.h>	//For memcpy.


#include <firmware/drivers/mss_spi/mss_spi.h> // For the MSS SPI functions
#include <firmware/MSS_C0_hw_platform.h> // Contains the address of the CORE_SPI instance for the driver.
#include "firmware/drivers/CoreSPI/core_spi.h"
#include "firmware/drivers/CoreSPI/corespi_regs.h"
#include "hal.h"
#include "task.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SS_DISABLE_GPIO_STATE 1
#define SS_ENABLE_GPIO_STATE  0

#define SPI_GPIO_SS_ENABLE(pin)   MSS_GPIO_set_output((pin), SS_ENABLE_GPIO_STATE)
#define SPI_GPIO_SS_DISABLE(pin)  MSS_GPIO_set_output((pin), SS_DISABLE_GPIO_STATE)


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBALS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
spi_instance_t core_spi[NUM_SPI_INSTANCES]; // Initialized by CoreSPI initialization function.
SemaphoreHandle_t core_lock[NUM_SPI_INSTANCES]; // Semaphores for the mutex locks. Seems to be that for short operations (e.g. even 80 ms), a mutex is good: http://www.openrtos.net/FreeRTOS_Support_Forum_Archive/December_2014/freertos_FreeRTOS_FatFs_Works_only_with_taskENTER_CRITICAL_5dc853ffj.html

//The base address for each CoreSPI peripheral.
addr_t core_base_addr[NUM_SPI_INSTANCES] = {	CORESPI_C0_0,
                                                CORESPI_C1_0,
												CORESPI_C1_1,
												CORESPI_C1_2,
												CORESPI_C1_3,
												CORESPI_C1_4,
												CORESPI_C1_5	};
//The length of each CoreSPI fifo.
uint16_t core_fifo_len[NUM_SPI_INSTANCES] = {	8,
												8,
												8,
												8,
												8,
												8,
												8	};

//SPI tempoary buffer
uint8_t spi_temp_buff[SPI_BUFF_SIZE];

// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int init_spi()
{
    int rc = 1;
    //MSS_SPI_init(&g_mss_spi0);
    for (int ix = 0; ix < NUM_SPI_INSTANCES; ix++)
    {
        core_lock[ix] = xSemaphoreCreateMutex();
        if (core_lock[ix] == NULL)
        {
            //rc = 0;
            //break; // Break out of this for loop.
        }
    }

    if (rc)
    {
    	for(int ix = 0; ix < NUM_SPI_INSTANCES-1; ix++){
		  // Initialize the core SPI instance. Make sure the fifo depth matches
		  // the value set in the Libero project
		  SPI_init(&core_spi[ix], core_base_addr[ix], core_fifo_len[ix]);

		  uint8_t reg=0;
		  reg = HAL_get_8bit_reg( (&core_spi[ix])->base_addr, CTRL1);


		  SPI_configure_master_mode(&core_spi[ix]);

		  reg = HAL_get_8bit_reg( (&core_spi[ix])->base_addr, CTRL1);

		  uint8_t a =0;
    	}
    }

    return rc;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
spi_instance_t * get_spi_instance(CoreSPIInstance_t core)
{
    return &core_spi[core];
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//void spi_configure_slave(CoreSPIInstance_t core, spi_slave_t slave, SPI_protocol_mode_t protocol_mode, SPI_pclk_div_t clk_rate, SPI_order_t data_xfer_order)
//{
//    SPI_configure(&core_spi[core], slave, protocol_mode, clk_rate, data_xfer_order);
//}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void spi_configure_gpio_ss(mss_gpio_id_t pin)
{
    MSS_GPIO_config( pin, MSS_GPIO_OUTPUT_MODE );
    MSS_GPIO_set_output( pin, SS_DISABLE_GPIO_STATE );
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void spi_transaction_block_write_with_toggle(CoreSPIInstance_t core, spi_slave_t slave, uint8_t * cmd_buffer, size_t cmd_size, uint8_t * wr_buffer, size_t wr_size)
{
//    SPI_enable(&core_spi[core]);
//    SPI_slave_select(&core_spi[core], slave);
//    SPI_block_write(&core_spi[core], cmd_buffer, cmd_size, wr_buffer, wr_size);
//    SPI_disable(&core_spi[core]);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void spi_transaction_block_read_with_toggle(CoreSPIInstance_t core, spi_slave_t slave, uint8_t * cmd_buffer, size_t cmd_size, uint8_t * rd_buffer, size_t rd_size)
{
//    SPI_enable(&core_spi[core]);
//    SPI_slave_select(&core_spi[core], slave);
//    SPI_block_read(&core_spi[core], cmd_buffer, cmd_size, rd_buffer, rd_size);
//    SPI_disable(&core_spi[core]);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void spi_transaction_block_write_without_toggle(CoreSPIInstance_t core, spi_slave_t slave, uint8_t * cmd_buffer, uint16_t cmd_size, uint8_t * wr_buffer, uint16_t wr_size)
{

	//Put the command and data into one buffer.
	uint32_t total_count = cmd_size + wr_size;

	if(total_count < SPI_BUFF_SIZE){

		//Copy the data into the static buffer.
		//We should not need to clear the buffer since we overwrite previous data and know the length.
        memcpy(spi_temp_buff,cmd_buffer,cmd_size);
        memcpy(&spi_temp_buff[cmd_size],wr_buffer,wr_size);


        if(core == MSS_SPI_0){
            //For the built-in MSS SPI peripheral.
            MSS_SPI_set_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);
            MSS_SPI_transfer_block(&g_mss_spi0,spi_temp_buff, total_count, 0, 0);
            MSS_SPI_clear_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);
        }
        else{
            //For the CoreSPI FPGA Cores.
            SPI_set_slave_select(&core_spi[core], slave);
            SPI_transfer_block(&core_spi[core],spi_temp_buff, total_count, 0, 0);
            SPI_clear_slave_select(&core_spi[core],slave);
        }
	}
	else{
		//Handle when the data exceeds the size of the static buffer.
		//Shouldn't really happen since we can increase size of the buffer
		//during development or modify the calling code..
	    while(1){}// Just loop here for now, until we add error code.
	}
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void spi_transaction_block_read_without_toggle(CoreSPIInstance_t core, spi_slave_t slave, uint8_t * cmd_buffer, uint16_t cmd_size, uint8_t * rd_buffer, uint16_t rd_size)
{
    if(core == MSS_SPI_0){
        MSS_SPI_set_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);
        MSS_SPI_transfer_block(&g_mss_spi0,cmd_buffer, cmd_size, rd_buffer, rd_size);
        MSS_SPI_clear_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);
    }
    else{
        SPI_set_slave_select(&core_spi[core], slave);
        SPI_transfer_block(&core_spi[core], cmd_buffer, cmd_size, rd_buffer, rd_size);
        SPI_clear_slave_select(&core_spi[core],slave);
    }
}
