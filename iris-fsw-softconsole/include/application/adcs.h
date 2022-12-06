/*
 * adcs.h
 *
 *  Created on: Dec. 5, 2022
 *      Author: jpmck
 */

#ifndef INCLUDE_APPLICATION_ADCS_H_
#define INCLUDE_APPLICATION_ADCS_H_

#include "main.h"
#include "application/telemetry_manager.h"

void HandleAdcsTask(TaskId_t req, uint8_t * params, Calendar_t * time);

#endif /* INCLUDE_APPLICATION_ADCS_H_ */
