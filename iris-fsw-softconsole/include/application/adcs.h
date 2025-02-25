/*
 * adcs.h
 *
 *  Created on: Dec. 5, 2022
 *      Author: jpmck
 */

#ifndef INCLUDE_APPLICATION_ADCS_H_
#define INCLUDE_APPLICATION_ADCS_H_

#include <application/memory_manager.h>
#include "main.h"
#include "drivers/device/adcs_driver.h"

void HandleAdcsCommand(int);
AdcsDriverError_t adcsArbCommand(uint8_t cmd_id, uint8_t* rxBuf, uint8_t expectedLen);
#endif /* INCLUDE_APPLICATION_ADCS_H_ */
