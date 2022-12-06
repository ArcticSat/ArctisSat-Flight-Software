/*
 * telemetry_manager.h
 *
 *  Created on: Dec. 2, 2022
 *      Author: jpmckoy
 */

#ifndef INCLUDE_APPLICATION_TELEMETRY_MANAGER_H_
#define INCLUDE_APPLICATION_TELEMETRY_MANAGER_H_

#include "tasks/telemetry.h"

#define EVENT_DATA_SIZE 4

typedef enum {
	SC_STATUS,
	EVENT_LOG,
	CDH_CHANNEL,
	POWER_TLM_CHANNEL,
	PAYLOAD_TLM_CHANNEL,
	ADCS_TLM_CHANNEL,
	NUM_TLM_CHANNELS,
} TelemetryChannel_t;

typedef enum
{
	TASK_ERROR,
	TASK_SUCCESS
} EventId_t;

void init_telemetry_manager(void);
void log_telemetry(telemetryPacket_t * pkt);
void get_telemetry(TelemetryChannel_t channel_id);

#endif /* INCLUDE_APPLICATION_TELEMETRY_MANAGER_H_ */
