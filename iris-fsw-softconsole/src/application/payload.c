/*
 * payload.c
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */

#include "application/payload.h"
#include "application/application.h"
#include "tasks/telemetry.h"


void HandlePayloadTlm(telemetryPacket_t * tm_pkt)
{
	switch(tm_pkt->telem_id)
	{
	case 0:
		break;
	default:
		break;
	}
}
