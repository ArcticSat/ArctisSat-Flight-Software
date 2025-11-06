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

void HandlePayloadTlm(csp_conn_t * conn, csp_packet_t * packet)
{
	
}
