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
#include "FreeRTOS.h"
#include "task.h"

void HandleAdcsCommand(telemetryPacket_t * cmd_pkt)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
	uint8_t tx_buf[10];
	uint8_t rx_buf[400];
	telemetryPacket_t tm_pkt = {0};
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
		// Send telemetry
		tm_pkt.telem_id = ADCS_MESAUREMENT_SUN_ID;
		/*
		tm_pkt.length = ADCS_SUN_SENSOR_DATA_SIZE;
		tm_pkt.data = &rx_buf[0];
		// Log telemetry
		log_telemetry(&tm_pkt);
		// Send telemetry
		tm_pkt.data = &rx_buf[164];
		// Log telemetry
		vTaskDelay(10);
		log_telemetry(&tm_pkt);*/

		// Debug
		tm_pkt.length = 10;
		// First 30 bytes of SS_X
		for(i=0; i < 3*10+1; i+=10){
			tm_pkt.data = &rx_buf[i];
			log_telemetry(&tm_pkt);
			vTaskDelay(10);
		}
//		// First 30 bytes of SS_X
//		for(i=0; i < 3*10+1; i+=10){
//			tm_pkt.data = &rx_buf[164+i];
//			log_telemetry(&tm_pkt);
//			vTaskDelay(10);
//		}
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
