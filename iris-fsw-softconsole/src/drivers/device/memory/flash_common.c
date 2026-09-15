//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Common interface for external flash drivers.
//
// History
// 2020-03-30 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "drivers/device/memory/flash_common.h"
#include "drivers/device/memory/W25N_flash.h"
#include "drivers/device/memory/W25Q_flash.h"
#include "drivers/protocol/spi.h"
#include "board_definitions.h"


//Device specific spi functions
void data_flash_spi_read(uint8_t *cmd_buffer,uint16_t cmd_size,uint8_t *rd_buffer,uint16_t rd_size);
void data_flash_spi_write(uint8_t *cmd_buffer,uint16_t cmd_size,uint8_t *wr_buffer,uint16_t wr_size);

void program_flash_spi_read(uint8_t *cmd_buffer, uint16_t cmd_size, uint8_t *rd_buffer, uint16_t rd_size);
void program_flash_spi_write(uint8_t *cmd_buffer,uint16_t cmd_size,uint8_t *wr_buffer,uint16_t wr_size);

#if defined(W25N_DATA_FLASH)
static W25NDevice_t data_flash_driver = {	.spi_read = data_flash_spi_read,
											.spi_write=data_flash_spi_write,
											.size = 1*W25N_DIE_SIZE};

static FlashDev_t 	data_flash = {	.driver = &data_flash_driver,
									.id = DATA_FLASH,
									.page_size = W25N_PAGE_SIZE,
									.erase_size = W25N_BLOCK_SIZE,
									.device_size = 1*W25N_DIE_SIZE	};
#endif

#if defined(W25Q_PROGRAM_FLASH)
static W25Q_Device_t program_flash_driver = {   .spi_read = program_flash_spi_read,
                                              .spi_write= program_flash_spi_write};


static FlashDev_t 	program_flash = {	.driver = &program_flash_driver,
										.id = PROGRAM_FLASH,
										.page_size = W25Q_PAGE_SIZE,
										.erase_size = W25Q_SUBSECTOR_SMALL_SIZE,
										.device_size = W25Q_DIE_SIZE	};

#endif



FlashDev_t *flash_devices[NUM_FLASH_DEVICES] = {
#ifdef USING_DATA_FLASH
		&data_flash,
#else
		NULL,
#endif
#ifdef USING_PROGRAM_FLASH
		&program_flash
#else
		NULL,
#endif
};


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
FlashStatus_t flash_device_init(FlashDev_t *device){
	FlashStatus_t result = FLASH_ERROR;
	switch(device->id){
		case DATA_FLASH:{
#if defined(W25N_DATA_FLASH)
			result = w25n_dev_init(device->driver, 0, ECC_ON);//Note: The reserved blocks should be subtracted from the device size, since the w25n driver can't update the flash device struct.
#endif
		    break;
		} // case: DATA_FLASH
		case PROGRAM_FLASH:{
#if defined(W25Q_PROGRAM_FLASH)
		    result = W25Q_setup_flash(device->driver);
#endif
			break;
		} // case: PROGRAM_FLASH
	} // switch(device->id)
	return result;
} // end of: flash_device_init

FlashStatus_t flash_write(FlashDev_t *device,uint32_t address, uint8_t *wr_buffer,uint32_t wr_size){

	FlashStatus_t result = FLASH_ERROR;
	switch(device->id){
		case DATA_FLASH:{
#if defined(W25N_DATA_FLASH)
			result = w25n_write(device->driver,address,wr_size,wr_buffer);
#endif
		    break;
		} // case: DATA_FLASH
		case PROGRAM_FLASH:{
#if defined(W25Q_PROGRAM_FLASH)
			result = W25Q_flash_write_page(device->driver, address, wr_buffer, wr_size);
#endif
			break;
		} // case: PROGRAM_FLASH
	} // switch(device->id)
	return result;
} // end of: flash_write

FlashStatus_t flash_read(FlashDev_t *device, uint32_t address, uint8_t *rd_buffer,uint32_t rd_size){

	FlashStatus_t result = FLASH_ERROR;
	switch(device->id){
		case DATA_FLASH:{
#if defined(W25N_DATA_FLASH)
			result = w25n_read(device->driver,address,rd_size,rd_buffer);
#endif
		    break;
		} // case: DATA_FLASH
		case PROGRAM_FLASH:{
#if defined(W25Q_PROGRAM_FLASH)
			result = W25Q_flash_read(device->driver, address, rd_buffer, rd_size);
#endif
			break;
		} // case PROGRAM_FLASH
	} // switch: device->id
	return result;
} // end of: flash_read

FlashStatus_t flash_erase(FlashDev_t *device, uint32_t address){

	FlashStatus_t result = FLASH_ERROR;
	switch(device->id){
		case DATA_FLASH:{
#if defined(W25N_DATA_FLASH)
		uint32_t blocknum = address/W25N_BLOCK_SIZE;
		result = w25n_erase_blocks(device->driver,blocknum,1);
#endif
	    break;
	} // case: DATA_FLASH
		case PROGRAM_FLASH:{
#if defined(W25Q_PROGRAM_FLASH)
			result = W25Q_flash_erase_4k(device->driver, address);
#endif
			break;
		} // case PROGRAM_FLASH
	} // switch: device->id
	return result;
} // end of: flash_erase

FlashStatus_t flash_erase_device(FlashDev_t *device){

	FlashStatus_t result = FLASH_ERROR;
	switch(device->id){
		case DATA_FLASH:{
#if defined(W25N_DATA_FLASH)
		uint32_t numblocks = W25N_DIE_SIZE/W25N_BLOCK_SIZE -8;
		result = w25n_erase_blocks(device->driver,0,numblocks);
#endif
		    break;
		} // case: DATA_FLASH
		case PROGRAM_FLASH:{
#if defined(W25Q_PROGRAM_FLASH)
			result = W25Q_flash_erase_device(device->driver);
#endif
			break;
		} // case PROGRAM_FLASH
	} // switch: device->id
	return result;
} // end of: flash_erase_device

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEVICE SPECIFIC FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//These are used so that we can easily modify the basic spi functions used for each driver.
//Each Flash driver ultimately just does spi read and spi write, so we write the drivers to use these functions.
//This makes it so we only have to change these in one place instead of all across the flash drivers.


void data_flash_spi_read(uint8_t *cmd_buffer,uint16_t cmd_size,uint8_t *rd_buffer,uint16_t rd_size){
	spi_transaction_block_read_without_toggle(FLASH_SPI_CORE, FLASH_SLAVE_CORE,  cmd_buffer, cmd_size, rd_buffer, rd_size);
}
void data_flash_spi_write(uint8_t *cmd_buffer,uint16_t cmd_size,uint8_t *wr_buffer,uint16_t wr_size){
	spi_transaction_block_write_without_toggle(FLASH_SPI_CORE, FLASH_SLAVE_CORE,  cmd_buffer, cmd_size, wr_buffer, wr_size);
}

void program_flash_spi_read(uint8_t *cmd_buffer, uint16_t cmd_size, uint8_t *rd_buffer, uint16_t rd_size){

    spi_transaction_block_read_without_toggle(FLASH2_SPI_CORE, FLASH_SLAVE_CORE, cmd_buffer, cmd_size, rd_buffer, rd_size);

}
void program_flash_spi_write(uint8_t *cmd_buffer,uint16_t cmd_size,uint8_t *wr_buffer,uint16_t wr_size){

    spi_transaction_block_write_without_toggle(FLASH2_SPI_CORE, FLASH_SLAVE_CORE, cmd_buffer, cmd_size, wr_buffer, wr_size);
}
