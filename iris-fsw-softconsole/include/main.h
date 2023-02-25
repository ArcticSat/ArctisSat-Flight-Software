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

typedef enum {
	CDH_SUBSYS_ID,
	POWER_SUBSYS_ID,
	PAYLOAD_SUBSYS_ID,
	ADCS_SUBSYS_ID,
	NUM_SUBSYSTEMS
} SubsystemId_t;

#endif /* INCLUDE_MAIN_H_ */