/*
 * application.c
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */


#include "application/application.h"
#include "application/payload.h"

bool ls_rst_status[NUM_LOAD_SWITCHES];

void HandleTm(telemetryPacket_t * tm_pkt)
{
	if(tm_pkt->telem_id < NUM_PAYLOAD_TELEMETRY)
		HandlePayloadTlm(tm_pkt);
}

bool GetLsRstStatus(uint8_t switchNumber)
{
	return ls_rst_status[switchNumber];
}

bool ResetLoadSwitch(uint8_t switchNumber)
{
	ls_rst_status[switchNumber] = false;
}

void InitApplication(void)
{
	int i;
	for(i=0; i < NUM_LOAD_SWITCHES; i++)
		ls_rst_status[i] = true;
}
