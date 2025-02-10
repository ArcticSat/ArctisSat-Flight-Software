/*
 * adcs.c
 *
 *  Created on: Dec. 5, 2022
 *      Author: jpmck
 */

#include "application/adcs.h"
#include "main.h"
#include "tasks/telemetry.h"
#include "drivers/device/adcs_driver.h"
#include "application/sun_pointing.h"
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/device/rtc/rtc_common.h"
//#include "drivers/mss_rtc/mss_rtc.h"
//#include "drivers/device/rtc/rtc_time.h"


Calendar_t * lowerBound;
Calendar_t * upperBound;

void HandleAdcsCommand(telemetryPacket_t * cmd_pkt)
{
	uint8_t tx_buf[10];
	uint8_t rx_buf[40] = {0};
	int i;
	int error_code = 10;
	// Perform Task
	switch(cmd_pkt->telem_id)
	{
	case ADCS_INIT_CMD:
		error_code = adcs_init_driver();
		break;
	case ADCS_TX_RX_CMD:
		// Parameters
		error_code = adcsTxRx(
				&cmd_pkt->data[2],				// Tx data
				(uint16_t) cmd_pkt->data[0],	// Tx length
				rx_buf,							// Rx data
				(uint16_t) cmd_pkt->data[1]);	// Rx length
		break;
	case ADCS_PING_CMD:
		error_code = pingAdcs();
		break;
	case ADCS_SPI_SYNC_CMD:
		error_code = adcsSyncSpi();
		break;
	case ADCS_GET_MEASUREMENT_GYRO_CMD:
	    i = cmd_pkt->data[0];
		error_code = getGyroMeasurementsRaw(0, rx_buf);
		break;
    case ADCS_DEPLOY_LEFT_CMD:
        i = cmd_pkt->data[0];
        error_code = adcsSyncSpiCommand(0x08);
        break;
    case ADCS_DEPLOY_RIGHT_CMD:
        i = cmd_pkt->data[0];
        error_code = adcsSyncSpiCommand(0x09);
        break;
    case ADCS_DEPLOY_BOTH_CMD:
        i = cmd_pkt->data[0];
        error_code = adcsSyncSpiCommand(0x0C);
        break;
    case ADCS_STOW_LEFT_CMD:
        i = cmd_pkt->data[0];
        error_code = adcsSyncSpiCommand(0x0A);
        break;
    case ADCS_STOW_RIGHT_CMD:
        i = cmd_pkt->data[0];
        error_code = adcsSyncSpiCommand(0x0B);
        break;
    case ADCS_STOW_BOTH_CMD:
        i = cmd_pkt->data[0];
        error_code = adcsSyncSpiCommand(0x0D);
        break;
	case ADCS_GET_MEASUREMENT_MAGNETOMETER_CMD:
		// Get reading
		error_code = getMagnetometerMeasurementsRaw(0, rx_buf);
		break;
	default:
		return;
	}
	// Logging
	telemetryPacket_t event = {0};
	event.telem_id = EVENT_ID;
	event.timestamp = cmd_pkt->timestamp;
	event.length = 2;
	if(error_code == ADCS_DRIVER_NO_ERROR)
	{
		event.data[0] = TASK_SUCCESS;
		event.data[1] = cmd_pkt->telem_id;
	}
	else
	{
		event.data[0] = TASK_ERROR;
		event.data[1] = cmd_pkt->telem_id;
	}
}
