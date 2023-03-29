/*
 * main.h
 *
 *  Created on: Dec. 2, 2022
 *      Author: jpmck
 */

#ifndef INCLUDE_MAIN_H_
#define INCLUDE_MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define FLIGHT_MODEL_CONFIGURATION
//#define ENGINEERING_MODEL_CONFIGURATION
//#define MAKER2_DEVKIT_CONFIGURATION

typedef enum {
	CDH_SUBSYS_ID,
	POWER_SUBSYS_ID,
	PAYLOAD_SUBSYS_ID,
	ADCS_SUBSYS_ID,
	NUM_SUBSYSTEMS
} SubsystemId_t;

typedef struct{
    uint8_t can_init;   //1 is success, 0 is failure(null queue)
    uint8_t spi_init;   //1 is success, 0 is failure(semaphore create failed)
    uint8_t data_flash_init; //0 is success, else error (check FlashStatus_t)
    uint8_t program_flash_init; //^^^
}HardwareCheck_t;

extern HardwareCheck_t setupHardwareStatus;

#endif /* INCLUDE_MAIN_H_ */
