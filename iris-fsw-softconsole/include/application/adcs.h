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

void HandleAdcsCommand(telemetryPacket_t * cmd_pkt);

#endif /* INCLUDE_APPLICATION_ADCS_H_ */
