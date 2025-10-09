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

#include "cdh_configuration.h"


typedef enum {
	CDH_SUBSYS_ID,
	POWER_SUBSYS_ID,
	PAYLOAD_SUBSYS_ID,
	ADCS_SUBSYS_ID,
	NUM_SUBSYSTEMS
} SubsystemId_t;

typedef struct{
    uint8_t can_init;   		//1 is success, 0 is failure(null queue)
    uint8_t spi_init;   		//1 is success, 0 is failure(semaphore create failed)
	uint8_t CSP_init;  			//1 is success, 0 is failure
    uint8_t data_flash_init; 	//0 is success, else error (check FlashStatus_t)
    uint8_t program_flash_init; //0 is success, else error (check FlashStatus_t)
	uint8_t rtc_init;  			//0 is success, else error (check MSS_R
	uint8_t mram_init; 			//0 is success, else error (check FlashStatus_t)
	uint8_t fs_init;   			//0 is success, else error (check FilesystemError_t)
	uint8_t uart_init; 			//0 is success, else error (check MSS_UART_Status)
	uint8_t wd_init;   			//0 is success, else error (check WD_Status)
} HardwareCheck_t;

typedef enum {
	SUBSYTEM_WARNING,
	SUBSYSTEM_ERROR,
	SUBSYSTEM_CRITICAL,
	SUBSYSTEM_LOST,
	SUBSYSTEM_NORMAL
} subsystemStatus_t;

typedef struct {
	subsystemStatus_t ADCS_status;
	subsystemStatus_t POWER_status;
	subsystemStatus_t PAYLOAD_status;
	subsystemStatus_t COMMS_status;

} continuousHardwareStatus_t;

extern HardwareCheck_t setupHardwareStatus;

#endif /* INCLUDE_MAIN_H_ */
