#ifndef SOFTWARE_UPDATE_H_
#define SOFTWARE_UPDATE_H_
//------------------------------------------------------------------------------
// File Description:
//  Driver for updating the software/FPGA fabric (ISP).
//
// History
// 2020-05-05 by Joseph Howarth
// - Created.
//------------------------------------------------------------------------------

#include <stdint.h>

//Program flash is 16MBit or 2MB, so we split roughly in 2.
//SPI directory goes at the begining. Firmware can be up to 1000kb.
#define FIRMWARE_GOLDEN_ADDRESS	0x001000
#define FIRMWARE_UPDATE_ADDRESS 0x100000

enum{
    FW_GOLDEN_INDEX,
    FW_UPDATE_INDEX,

};

//Will compare the current running firmware with a fw file on the program flash.
//Parameter:
//  version: 0 for golden image, 1 for update image.
//  design_ver: pointer where the design version of the running firmware will be placed.
//returns 0 if ok, error code otherwise.
int authenticate_firmware(uint8_t version, uint16_t* design_ver);

void initiate_firmware_update(uint8_t version);

//This will update the spi dir file used in iap to have the correct design version for the golden/upgrade file.
//The  adressses should never change, so those are hard coded.
void update_spi_dir(uint8_t version, uint16_t design_ver);
void save_program(void * buff, uint16_t size, uint8_t version);//version = 0 for golden image, version = 1 for updated image.
void repeat_program();
void set_program_size(uint32_t size,uint8_t version);
void get_spi_dir(uint8_t* result);
void create_spi_dir(uint8_t * dir, uint8_t length);
uint16_t get_design_version();

#endif
