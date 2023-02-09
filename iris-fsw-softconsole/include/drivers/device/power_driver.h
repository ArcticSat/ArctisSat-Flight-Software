/*
 * power_driver.h
 *
 *  Created on: Jan 29, 2022
 *      Author: mckoyj
 */

#ifndef INCLUDE_DRIVERS_DEVICE_POWER_DRIVER_H_
#define INCLUDE_DRIVERS_DEVICE_POWER_DRIVER_H_

// Not sure which includes are actually necessary
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include "drivers/device/rtc/rtc_common.h"
#include "tasks/telemetry.h"

#define CDH_CAN_ID 		0x721
#define POWER_CAN_ID 	0x824

void handlePowTelemetry(uint8_t * tlm_data);

#endif /* INCLUDE_DRIVERS_DEVICE_POWER_DRIVER_H_ */
