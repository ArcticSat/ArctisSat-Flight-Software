/*
 * telemetry_manager.h
 *
 *  Created on: Dec. 2, 2022
 *      Author: jpmckoy
 */

#ifndef INCLUDE_APPLICATION_TELEMETRY_MANAGER_H_
#define INCLUDE_APPLICATION_TELEMETRY_MANAGER_H_

#include "tasks/telemetry.h"

void init_telemetry_manager(void);
void log_telemetry(telemetryPacket_t * pkt);
void get_telemetry(uint8_t tlm_blk_id);

#endif /* INCLUDE_APPLICATION_TELEMETRY_MANAGER_H_ */
