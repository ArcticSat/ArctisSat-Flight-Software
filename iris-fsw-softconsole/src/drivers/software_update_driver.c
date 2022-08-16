//------------------------------------------------------------------------------
// File Description:
//  Driver for updating the software/FPGA fabric (ISP).
//
// History
// 2020-05-05 by Joseph Howarth
// - Created.
//------------------------------------------------------------------------------

#include "drivers/software_update_driver.h"
#include "drivers/device/memory/flash_common.h"
//#include <firmware/drivers/mss_uart/mss_uart.h>
//#include "drivers/protocol/uart.h"
#include <firmware/drivers/mss_sys_services/mss_sys_services.h>
#include <string.h>
#include <firmware/drivers/mss_spi/mss_spi.h>
#include "FreeRTOS.h"


static uint32_t programSize = 0;
static uint8_t initialized = 0;

uint16_t get_design_version(){

    uint8_t stat = 255;
    if(!initialized){
        MSS_SYS_init(MSS_SYS_NO_EVENT_HANDLER);
        initialized = 1;
    }
    uint16_t dv = 0xFFFF;
    stat = MSS_SYS_get_design_version((uint8_t*)&dv);

    return dv;
}
int authenticate_firmware(uint8_t version, uint16_t *design_ver){

	uint32_t addr = (version)?FIRMWARE_UPDATE_ADDRESS:FIRMWARE_GOLDEN_ADDRESS;
	uint8_t stat = 255;
	if(!initialized){
		MSS_SYS_init(MSS_SYS_NO_EVENT_HANDLER);
		initialized = 1;
	}
	uint16_t dv = 0xFFFF;
	stat = MSS_SYS_get_design_version((uint8_t*)&dv);

	printf("Authenticating Firmware...Found design ver. %d\n",dv);

	*design_ver = dv;

	MSS_SPI_set_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);

	uint8_t result = MSS_SYS_initiate_iap(MSS_SYS_PROG_VERIFY,addr);
	MSS_SPI_clear_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);

	if(result == MSS_SYS_SUCCESS){
		printf("Firmware authenticated!\n");
	}else{
		printf("Firmware Authentication Failed: %d\n",result);
	}
	return result ;
}

void initiate_firmware_update(uint8_t version){

	uint32_t addr = (version)?FIRMWARE_UPDATE_ADDRESS:FIRMWARE_GOLDEN_ADDRESS;
	if(!initialized){
		MSS_SYS_init(MSS_SYS_NO_EVENT_HANDLER);
		initialized = 1;
	}

	printf("Initiating Firmware Upgrade (IAP)...\n");
	printf("System will reboot in 5 seconds!\n");
	vTaskDelay(5000);

	MSS_SPI_set_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);
	uint8_t result = MSS_SYS_initiate_iap(MSS_SYS_PROG_AUTHENTICATE,addr);
	result = MSS_SYS_initiate_iap(MSS_SYS_PROG_PROGRAM,addr);
	MSS_SPI_clear_slave_select(&g_mss_spi0,MSS_SPI_SLAVE_0);

	//If we get here the IAP failed...
	printf("Firmware Upgrade Failed: System did not reboot (%d)\n",result);

}

void repeat_program(uint32_t version){

//	uint32_t numTransmits = programSize/512;
//	uint32_t remaining = programSize%512;
//	uint8_t buff[512];
//
//	uint32_t index = (version)?FIRMWARE_GOLDEN_ADDRESS:FIRMWARE_UPDATE_ADDRESS;
//	for(int i=0;i<numTransmits;i++){
//
//		flash_read(flash_devices[PROGRAM_FLASH], index, buff, 512);
//		prvUARTSend(&g_mss_uart0, buff, 512);
//		index += 512;
//	}
//
//	flash_read(flash_devices[PROGRAM_FLASH], index, buff, remaining);
//	prvUARTSend(&g_mss_uart0, buff, remaining);

}
void set_program_size(uint32_t size, uint8_t version){
//	programSize = size;
//	flash_device_init(flash_devices[PROGRAM_FLASH]);
//
//	if(version == 0){
//		uint32_t start = FIRMWARE_UPDATE_ADDRESS-1;
//		while(start < flash_devices[PROGRAM_FLASH]->device_size){
//			flash_erase(flash_devices[PROGRAM_FLASH], start);
//			start += flash_devices[PROGRAM_FLASH]->erase_size;
//		}
//	}
//	else{
//		uint32_t start = FIRMWARE_GOLDEN_ADDRESS-1;
//		while(start < FIRMWARE_UPDATE_ADDRESS){
//			flash_erase(flash_devices[PROGRAM_FLASH], start);
//			start += flash_devices[PROGRAM_FLASH]->erase_size;
//		}
//
//	}
//
//	uint8_t spi_dir[13] = {0x00, 0x01,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x00,0x03,0x00,0x0a};
//	FlashStatus_t stat= flash_write(flash_devices[PROGRAM_FLASH], 0, spi_dir, sizeof(spi_dir));

}

void update_spi_dir(uint8_t  version, uint16_t design_ver){

    //Little endian...

//    uint8_t spi_dir[13] = {0x00, 0x10,0x00,0x00, //Golden address (0x1000)
//                           0x00,0x00,            //Golden Design Ver (0)
//                           0x00,0x00,0x10,0x00,  //Update addrress (default 0x100000)
//                           0x01,0x00,            //Update design ver
//                           0x0a};                //File End (Linefeed)

	uint8_t spi_dir[13]= {0};
	flash_read(flash_devices[PROGRAM_FLASH],0,spi_dir,13);
    uint8_t location = (version ? 10:4); //Insert to byte 10 if update image(1), or byte 4 for godlen image(0).
    memcpy(&spi_dir[location],&design_ver,2);
    FlashStatus_t stat= flash_erase(flash_devices[PROGRAM_FLASH], 0);
    stat= flash_write(flash_devices[PROGRAM_FLASH], 0, spi_dir, sizeof(spi_dir));

}

void create_spi_dir(uint8_t * dir, uint8_t length){

	//This is the template.. This function should only be used for debug purpose.
//    //Little endian...
//    uint8_t spi_dir[13] = {0x00, 0x10,0x00,0x00, //Golden address (0x1000)
//                           0x00,0x00,            //Golden Design Ver (0)
//                           0x00,0x00,0x10,0x00,  //Update addrress (default 0x100000)
//                           0x01,0x00,            //Update design ver
//                           0x0a};                //File End (Linefeed)

    FlashStatus_t stat= flash_erase(flash_devices[PROGRAM_FLASH], 0);
    stat= flash_write(flash_devices[PROGRAM_FLASH], 0, dir, length);

}

void save_program(void * buff, uint16_t size, uint8_t version){
//OBSOLETE
//	static uint32_t pos_golden = FIRMWARE_UPDATE_ADDRESS;
//	static uint32_t pos_update = FIRMWARE_GOLDEN_ADDRESS;
//
//
//	if(version == 0){
//		FlashStatus_t stat= flash_write(flash_devices[PROGRAM_FLASH], pos_golden, buff, size);
//		if(stat == FLASH_OK) pos_golden += size;
//	}
//	else if(version == 1){
//		FlashStatus_t stat= flash_write(flash_devices[PROGRAM_FLASH], pos_update, buff, size);
//		if(stat == FLASH_OK) pos_update += size;
//	}
}

void get_spi_dir(uint8_t* result){

    flash_read(flash_devices[PROGRAM_FLASH],0,result,13);

}
