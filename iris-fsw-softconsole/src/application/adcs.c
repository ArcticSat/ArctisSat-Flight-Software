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

AdcsDriverError_t adcsArbCommand(uint8_t cmd_id, uint8_t* rxBuf, uint16_t expectedLen)
{
    while(adcsSyncSpiCommand(cmd_id));
    vTaskDelay(200);
    return adcsTxRx(NULL,0,rxBuf,expectedLen);

}

void HandleAdcsCommand(int cmd_id)
{
	uint8_t tx_buf[10];
	uint8_t rx_buf[40] = {0};
	int i;
	int error_code = 10;

	return;
	// Perform Task
	switch(cmd_id)
	{
	case ADCS_INIT_CMD:
		error_code = adcs_init_driver();
		break;
	case ADCS_TX_RX_CMD:
		// Parameters
	case ADCS_PING_CMD:
		error_code = pingAdcs();
		break;
	case ADCS_SPI_SYNC_CMD:
		error_code = adcsSyncSpi();
		break;
	case ADCS_GET_MEASUREMENT_GYRO_CMD:
		error_code = getGyroMeasurementsRaw(0, rx_buf);
		break;
    case ADCS_DEPLOY_LEFT_CMD:
        error_code = adcsSyncSpiCommand(0x08);
        break;
    case ADCS_DEPLOY_RIGHT_CMD:
        error_code = adcsSyncSpiCommand(0x09);
        break;
    case ADCS_DEPLOY_BOTH_CMD:
        error_code = adcsSyncSpiCommand(0x0C);
        break;
    case ADCS_STOW_LEFT_CMD:
        error_code = adcsSyncSpiCommand(0x0A);
        break;
    case ADCS_STOW_RIGHT_CMD:
        error_code = adcsSyncSpiCommand(0x0B);
        break;
    case ADCS_STOW_BOTH_CMD:
        error_code = adcsSyncSpiCommand(0x0D);
        break;
	case ADCS_GET_MEASUREMENT_MAGNETOMETER_CMD:
		// Get reading
		error_code = getMagnetometerMeasurementsRaw(0, rx_buf);
		break;
	default:
	    adcsSyncSpiCommand(cmd_id);
		return;
	}
}
