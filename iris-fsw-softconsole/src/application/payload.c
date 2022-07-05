/*
 * payload.c
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */
#include <stdio.h>
#include "application/payload.h"
#include "application/application.h"
#include "tasks/telemetry.h"

typedef enum
{
	CAMERA_SENSOR_INIT_ERROR,
	START_JPEG_MODE_ERROR,
	DO_CAPTURE_ERROR,
	SIZE_THRESHOLD_ERROR,
	PIXEL_AVG_THRESHOLD_ERROR,
} ImageCaptureError_t ;

void HandlePayloadTlm(telemetryPacket_t * tm_pkt)
{
	switch(tm_pkt->telem_id)
	{
	// Send error to virtual GS/Log error
	case PAYLOAD_IMAGE_CAPTURE_ERROR_ID:{
		char msg[50];
		uint8_t error_id = tm_pkt->data[0];
		switch(error_id)
		{
		case CAMERA_SENSOR_INIT_ERROR:{
			sprintf(msg,"Camera init error");
			printMsg(msg);
			break;
		}
		case START_JPEG_MODE_ERROR:{
			sprintf(msg,"Camera init error");
			printMsg(msg);
			break;
		}
		case DO_CAPTURE_ERROR:{
			sprintf(msg,"Camera not responding. Capture terminated.");
			printMsg(msg);
			break;
		}
		case SIZE_THRESHOLD_ERROR:{
			sprintf(msg,"Image size below threshold.");
			printMsg(msg);
			break;
		}
		default:
			break;
		}
		// Reset Payload, resend command

		break;
	} // (tm_pkt->telem_id) end of case: PAYLOAD_IMAGE_CAPTURE_ERROR_ID
	default:
		break;
	} // (tm_pkt->telem_id) end of case: default
}
