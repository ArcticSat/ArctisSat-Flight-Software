/*
 * eps.h
 *
 *  Created on: Dec. 6, 2022
 *      Author: jpmck
 */

#ifndef INCLUDE_APPLICATION_EPS_H_
#define INCLUDE_APPLICATION_EPS_H_

#include "tasks/telemetry.h"

#define POW_RXID 0x721
#define POW_TXID 0x824

void HandlePowerCommand(telemetryPacket_t * cmd_pkt);

#endif /* INCLUDE_APPLICATION_EPS_H_ */
