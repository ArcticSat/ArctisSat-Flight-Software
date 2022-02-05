/*
 * power_driver.c
 *
 *  Created on: Jan 29, 2022
 *      Author: mckoyj
 */

#include "drivers/device/power_driver.h"

void handlePowTelemetry(uint8_t * tlm_data)
{
	// Get telemetry ID fo switch statement
	uint8_t tlm_id = tlm_data[0];
	// "Unpack" telemetry data
	// Currently missing MSB for some reason (probably POW CAN send side)
	uint8_t pow_data[4] = {0};
	int i;
	for(i=0; i < 4; i++)
	{
		pow_data[i] = tlm_data[i+1];
	}
	// Handle data received
	switch(tlm_id)
	{
		case POWER_READ_TEMP_ID:{
			telemetryPacket_t packet;
			Calendar_t timestamp = {0}; // timestamp shouldn't matter
			packet.timestamp = timestamp;
			packet.telem_id = 13; // currently payload_msg_id...
			packet.length = 4; // for now, since we're sending floats
			packet.data = pow_data;
			sendTelemetryAddr(&packet, GROUND_CSP_ADDRESS);
			break;
		}
	}
}
