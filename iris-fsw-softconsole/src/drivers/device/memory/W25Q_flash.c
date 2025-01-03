//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//   Program flash memory chip on the IrisSat CDH board.
//
// History
// 2021-03-04 by Joseph Howarth
// - Created.
//
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "drivers/device/memory/W25Q_flash.h"
#include "board_definitions.h"
#include <firmware/drivers/mss_spi/mss_spi.h>
#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "drivers/mss_gpio/mss_gpio.h"

//Returns 1 if device is busy, 0 if not.
uint8_t W25Q_is_busy(W25Q_Device_t * dev);

FlashStatus_t W25Q_setup_flash(W25Q_Device_t * dev){

	FlashStatus_t result = FLASH_OK;

	//Set up SPI driver.
	MSS_SPI_init(&g_mss_spi0);

	//Clock div of 256 to get 250khz SPI clock rate.
	MSS_SPI_configure_master_mode(&g_mss_spi0,MSS_SPI_SLAVE_0,MSS_SPI_MODE0,256,MSS_SPI_BLOCK_TRANSFER_FRAME_SIZE);



	//Init GPIO
	 MSS_GPIO_config( PROGRAM_FLASH_WP_PIN, MSS_GPIO_OUTPUT_MODE );
	 MSS_GPIO_config( PROGRAM_FLASH_HOLD_PIN, MSS_GPIO_OUTPUT_MODE );

	//set gpio high
	 MSS_GPIO_set_output(PROGRAM_FLASH_WP_PIN, 1);
	 MSS_GPIO_set_output(PROGRAM_FLASH_HOLD_PIN, 1);

	//Read the device ID.
	uint8_t command = W25Q_OP_READ_ID;
	uint8_t id[3] = {0};
	dev->spi_read(&command, sizeof(command),id,sizeof(id));

	if((id[0] != W25Q_ID_BYTE1) && (id[1] != W25Q_ID_BYTE2) && (id[2] != W25Q_ID_BYTE3)){
		result = FLASH_INVALID_ID;
	}

	return result;
}

FlashStatus_t W25Q_flash_write_page(W25Q_Device_t * dev,uint32_t addr, uint8_t* data,uint32_t size){

	if(addr>W25Q_DIE_SIZE) return FLASH_INVALID_ADDRESS;

	uint8_t we_command = W25Q_OP_WRITE_ENABLE;
	dev->spi_write(&we_command,sizeof(we_command),NULL,0);

	uint8_t command [4] = {	W25Q_OP_PROGRAM,
							(addr>>16) & 0xFF,
							(addr>>8)  & 0xFF,
							(addr)	   & 0xFF
							};

	dev->spi_write(command,sizeof(command),data,size);

	while(W25Q_is_busy(dev)){
        if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
            vTaskDelay(pdMS_TO_TICKS(2));
        }
	}

	return FLASH_OK;
}

FlashStatus_t W25Q_flash_read(W25Q_Device_t * dev,uint32_t addr, uint8_t* data,uint32_t size){

	if(addr > W25Q_DIE_SIZE) return FLASH_INVALID_ADDRESS;

	// Using the 0x0B command results in one dummy byte clocked after the
	// command, so we added a 0 to the command, so that the dummy byte is not
	// added to the data output.

		uint8_t command [5] = {	W25Q_OP_READ,
								(addr>>16) & (0xFF),
								(addr>>8)  & (0xFF),
								(addr)	   & (0xFF),
								0x00
								};

		dev->spi_read(command, sizeof(command),data,size);



	return FLASH_OK;
}

FlashStatus_t W25Q_flash_erase_device(W25Q_Device_t * dev){

	uint8_t we_command = W25Q_OP_WRITE_ENABLE;
	dev->spi_write(&we_command,sizeof(we_command),NULL,0);

	uint8_t command = W25Q_OP_ERASE_DIE;

	dev->spi_write(&command,sizeof(command),NULL,0);

	while(W25Q_is_busy(dev)){
        if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
            vTaskDelay(pdMS_TO_TICKS(100));
        }
	}

	return FLASH_OK;
}

FlashStatus_t W25Q_flash_erase_4k(W25Q_Device_t * dev,uint32_t addr){

	if(addr>W25Q_DIE_SIZE) return FLASH_INVALID_ADDRESS;

	uint8_t we_command = W25Q_OP_WRITE_ENABLE;
	dev->spi_write(&we_command,sizeof(we_command),NULL,0);

	uint8_t command [4] = {	W25Q_OP_ERASE_SECTOR_4k,
							(addr>>16) & 0xFF,
							(addr>>8)  & 0xFF,
							(addr)	   & 0xFF
							};

	dev->spi_write(command,sizeof(command),NULL,0);

	while(W25Q_is_busy(dev)){
        if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
            vTaskDelay(pdMS_TO_TICKS(100));
        }
	}

	return FLASH_OK;
}

uint8_t W25Q_is_busy(W25Q_Device_t * dev){

	uint8_t result = 0;
	uint8_t stat_reg = 0;
	uint8_t command = W25Q_OP_READ_STAT_REG_1;

	dev->spi_read(&command,sizeof(command),&stat_reg,sizeof(stat_reg));

	result = stat_reg & 0x01;	//Bit 0 of the  status register byte 1 is the ready/busy status.

	return result;
}
