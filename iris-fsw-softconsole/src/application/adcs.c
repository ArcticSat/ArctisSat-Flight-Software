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

void HandleAdcsTask(TaskId_t req, uint8_t * params, Calendar_t * time)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
	uint8_t buf[164];
	telemetryPacket_t pkt = {0};
	// Perform Task
	switch(req)
	{
	case TASK_ADCS_INIT:
		error_code = adcs_init_driver();
		break;
	case TASK_ADCS_ADCS_TX_RX:
		error_code = adcsTxRx(
				(uint8_t *) {0x11,0x22,0x33},
				3,
				buf,
				2);
		break;
	case TASK_ADCS_PING:
		error_code = pingAdcs();
		break;
	case TASK_ADCS_SPI_SYNC:
		error_code = adcsSyncSpi();
		break;
	case TASK_ADCS_SET_TR_STATE:
		error_code = setTorqueRodState(
				(TortqueRodId_t) params[0],
				(TortqueRodState_t) params[1]);
		break;
	case TASK_ADCS_SET_TR_POLARITY:
		error_code = setTorqueRodPolarity(
				(TortqueRodId_t) params[0],
				(TortqueRodPolarity_t) params[1]);
//		break;
	case TASK_ADCS_SET_TR_PWM:
		error_code = setTorqueRodPwm(
				(TortqueRodId_t) params[0],
				params[1]);
		break;
	case TASK_ADCS_GET_MEASUREMENT_GYRO:
		// Get reading
		error_code = getGyroMeasurements(
				(GyroId_t) params[0],
				buf);
		// Send telemetry
		pkt.telem_id = ADCS_MESAUREMENT_GYRO_ID;
		pkt.length = ADCS_GYRO_DATA_SIZE;
		pkt.data = buf;
		break;
	case TASK_ADCS_GET_MEASUREMENT_MAGNETOMETER:
		// Get reading
		error_code = getMagnetometerMeasurements(
				(MagnetometerId_t) params[0],
				buf);
		// Send telemetry
		pkt.telem_id = ADCS_MESAUREMENT_MAGNETOMETER_ID;
		pkt.length = ADCS_MAGNETORQUER_DATA_SIZE;
		pkt.data = buf;
		break;
	case TASK_ADCS_GET_MEASUREMENT_SUN:
		// Get reading
		error_code = getSunSensorMeasurements(buf);
		// Send telemetry
		pkt.telem_id = ADCS_MESAUREMENT_SUN_ID;
		pkt.length = ADCS_SUN_SENSOR_DATA_SIZE;
		pkt.data = buf;
		break;
	default:
		return;
	}
	// Logging
	telemetryPacket_t event = {0};
	event.telem_id = EVENT_ID;
	event.timestamp = *time;
	event.length = 2;
	if(error_code == ADCS_DRIVER_NO_ERROR)
	{
		event.data[0] = TASK_SUCCESS;
		event.data[1] = req;
	}
	else
	{
		event.data[0] = TASK_ERROR;
		event.data[1] = req;
	}

}
