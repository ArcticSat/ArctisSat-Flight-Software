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

void HandleAdcsCommand(telemetryPacket_t * cmd_pkt)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
	uint8_t tx_buf[10];
	uint8_t rx_buf[164];
	telemetryPacket_t tm_pkt = {0};
	// Perform Task
	switch(cmd_pkt->telem_id)
	{
	case ADCS_INIT_CMD:
		error_code = adcs_init_driver();
		break;
	case ADCS_ADCS_TX_RX_CMD:
		tx_buf[0] = 0x11;
		tx_buf[1] = 0x22;
		tx_buf[2] = 0x33;
		error_code = adcsTxRx(
				tx_buf,
				3,
				rx_buf,
				2);
		break;
	case ADCS_PING_CMD:
		error_code = pingAdcs();
		break;
	case ADCS_SPI_SYNC_CMD:
		error_code = adcsSyncSpi();
		break;
	case ADCS_SET_TR_STATE_CMD:
		error_code = setTorqueRodState(
				(TortqueRodId_t) cmd_pkt->data[0],
				(TortqueRodState_t) cmd_pkt->data[1]);
		break;
	case ADCS_SET_TR_POLARITY_CMD:
		error_code = setTorqueRodPolarity(
				(TortqueRodId_t) cmd_pkt->data[0],
				(TortqueRodPolarity_t) cmd_pkt->data[1]);
		break;
	case ADCS_SET_TR_PWM_CMD:
		error_code = setTorqueRodPwm(
				(TortqueRodId_t) cmd_pkt->data[0],
				cmd_pkt->data[1]);
		break;
	case ADCS_GET_MEASUREMENT_GYRO_CMD:
		// Get reading
		error_code = getGyroMeasurements(
				(GyroId_t) cmd_pkt->data[0],
				rx_buf);
		// Send telemetry
		tm_pkt.telem_id = ADCS_MESAUREMENT_GYRO_ID;
		tm_pkt.length = ADCS_GYRO_DATA_SIZE;
		tm_pkt.data = rx_buf;
		// Debugging
		sendTelemetryAddr(&tm_pkt, GROUND_CSP_ADDRESS);
		break;
	case ADCS_GET_MEASUREMENT_MAGNETOMETER_CMD:
		// Get reading
		error_code = getMagnetometerMeasurements(
				(MagnetometerId_t) cmd_pkt->data[0],
				rx_buf);
		// Send telemetry
		tm_pkt.telem_id = ADCS_MESAUREMENT_MAGNETOMETER_ID;
		tm_pkt.length = ADCS_MAGNETORQUER_DATA_SIZE;
		tm_pkt.data = rx_buf;
		// Debugging
		sendTelemetryAddr(&tm_pkt, GROUND_CSP_ADDRESS);
		break;
	case ADCS_GET_MEASUREMENT_SUN_CMD:
		// Get reading
		error_code = getSunSensorMeasurements(rx_buf);
		// Send telemetry
		tm_pkt.telem_id = ADCS_MESAUREMENT_SUN_ID;
		tm_pkt.length = ADCS_SUN_SENSOR_DATA_SIZE;
		tm_pkt.data = rx_buf;
		// Debugging
		sendTelemetryAddr(&tm_pkt, GROUND_CSP_ADDRESS);
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
