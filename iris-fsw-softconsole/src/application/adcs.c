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

void HandleAdcsCommand(telemetryPacket_t * cmd_pkt)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
	uint8_t tx_buf[10];
	uint8_t rx_buf[400] = {0};
	telemetryPacket_t tm_pkt = {0};
	uint8_t pkt_data[40];
	int i;
	// Perform Task
	switch(cmd_pkt->telem_id)
	{
	case ADCS_INIT_CMD:
		error_code = adcs_init_driver();
		break;
	case ADCS_TX_RX_CMD:
		// Parameters
//		uint8_t txlen = cmd_pkt->data[0];
//		uint8_t rxlen = cmd_pkt->data[1];
//		error_code = adcsTxRx(
//				&cmd_pkt->data[2],
//				(uint16_t) txlen,
//				rx_buf,
//				(uint16_t) rxlen);
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
	case ADCS_SET_TR_STATE_CMD:
		error_code = setTorqueRodState(
				(TorqueRodId_t) cmd_pkt->data[0],
				(TortqueRodState_t) cmd_pkt->data[1]);
		break;
	case ADCS_SET_TR_POLARITY_CMD:
//		error_code = setTorqueRodPolarity(
//				(TorqueRodId_t) cmd_pkt->data[0],
//				(TortqueRodPolarity_t) cmd_pkt->data[1]);
		error_code = setTorqueRodPolarity(
				(TorqueRodId_t) cmd_pkt->data[0],
				cmd_pkt->data[1]);
		break;
	case ADCS_SET_TR_PWM_CMD:
		error_code = setTorqueRodPwm(
				(TorqueRodId_t) cmd_pkt->data[0],
				cmd_pkt->data[1]);
		break;
	case ADCS_SET_GYRO_I2C_ADDRESS_CMD:
		error_code = setGyroI2cAddress(cmd_pkt->data[0]);
		break;
	case ADCS_GET_MEASUREMENT_GYRO_GENERIC_CMD:
		// Get reading
		error_code = getGyroMeasurementsGenericRaw(rx_buf);
		// Send telemetry
		tm_pkt.telem_id = ADCS_MESAUREMENT_GYRO_ID;
		tm_pkt.length = ADCS_GYRO_RAW_DATA_SIZE_BYTES;
		tm_pkt.data = rx_buf;
		// Log telemetry
		log_telemetry(&tm_pkt);
		break;
	case ADCS_GET_MEASUREMENT_GYRO_CMD:
		// Get reading
		error_code = getGyroMeasurementsRaw(
				(GyroId_t) cmd_pkt->data[0],
				rx_buf);
		// Send telemetry
		tm_pkt.telem_id = ADCS_MESAUREMENT_GYRO_ID;
		tm_pkt.length = ADCS_GYRO_RAW_DATA_SIZE_BYTES;
		tm_pkt.data = rx_buf;
		// Log telemetry
		log_telemetry(&tm_pkt);
		break;
	case ADCS_GET_MEASUREMENT_MAGNETOMETER_CMD:
		// Get reading
		error_code = getMagnetometerMeasurementsRaw(
				(MagnetometerId_t) cmd_pkt->data[0],
				rx_buf);
		// Send telemetry
		tm_pkt.telem_id = ADCS_MESAUREMENT_MAGNETOMETER_ID;
		tm_pkt.length = ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES;
		tm_pkt.data = rx_buf;
		// Log telemetry
		log_telemetry(&tm_pkt);
		break;
	case ADCS_SUN_SENSOR_SELECT_CMD:
		sunSensorSelect((enumSunSensor) cmd_pkt->data[0]);
		break;
	case ADCS_GET_MEASUREMENT_SUN_CMD:
		// Get reading
		error_code = getSunSensorMeasurementsRaw(rx_buf);
		// Convert
		float angle = AngleDecompose(rx_buf,3);
		memcpy(&rx_buf[12],&angle,sizeof(angle));
		// Send meta data
		tm_pkt.telem_id = ADCS_SS_META_ID;
		tm_pkt.length = 12;
		tm_pkt.data = rx_buf;
		sendTelemetryAddr(&tm_pkt, GROUND_CSP_ADDRESS);
		// Send raw telemetry
		tm_pkt.telem_id = ADCS_SS_RAW_ID;
		tm_pkt.length = 40;
		tm_pkt.data = pkt_data;
		for(i=12; i < 160; i+=40){
			memcpy(pkt_data,&rx_buf[i],40);
			sendTelemetryAddr(&tm_pkt, GROUND_CSP_ADDRESS);
		}
		// Send angle telemetry
		tm_pkt.telem_id = ADCS_SS_ANGLE_ID;
		tm_pkt.length = 4;
		tm_pkt.data = pkt_data;
		memcpy(&angle,pkt_data,sizeof(angle));
		sendTelemetryAddr(&tm_pkt, GROUND_CSP_ADDRESS);
		break;
	case ADCS_GET_SUN_POINTING_TM_CMD:
		SendSunPointingTelemetry();
		break;
	case ADCS_GET_ECLIPSE_TIME_CMD:
		getEclipseBounds(&pkt_data[0],&pkt_data[sizeof(Calendar_t)]);
//		// Send Telemetry
		tm_pkt.telem_id = ADCS_GET_ECLIPSE_TIME_ID;
		tm_pkt.length = 2*sizeof(Calendar_t);
		tm_pkt.data = pkt_data;
		sendTelemetryAddr(&tm_pkt, GROUND_CSP_ADDRESS);
		break;
	case ADCS_SET_ECLIPSE_TIME_CMD:
		setEclipseBounds((Calendar_t *) &tm_pkt.data[0],(Calendar_t *) &tm_pkt.data[sizeof(Calendar_t)])
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
