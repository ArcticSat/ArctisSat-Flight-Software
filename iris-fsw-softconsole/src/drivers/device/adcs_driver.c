//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Driver for communicating with the ADCS controller.
//
// History
// 2020-01-02 by Joseph Howarth
// - Created.
//
//  Based on information from https://github.com/aminya/IrisSat-ADCS-Software/issues/2 and standard spi implementation.
//  * This does not currently implement communication as specified in SIGMA operations manual *
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "drivers/device/adcs_driver.h"
#include "board_definitions.h"

#define ADCS_ACK_PREFIX 0x01

// ADCS Command IDs
typedef enum{
	ADCS_CMD_PING = 1,
	ADCS_CMD_SET_PWM_TORQUE_ROD_1,				// 2
	ADCS_CMD_SET_PWM_TORQUE_ROD_2,				// 3
	ADCS_CMD_SET_PWM_TORQUE_ROD_3,				// 4
	ADCS_CMD_SET_ON_TORQUE_ROD_1,				// 5
	ADCS_CMD_SET_ON_TORQUE_ROD_2,				// 6
	ADCS_CMD_SET_ON_TORQUE_ROD_3,				// 7
	ADCS_CMD_SET_OFF_TORQUE_ROD_1,				// 8
	ADCS_CMD_SET_OFF_TORQUE_ROD_2,				// 9
	ADCS_CMD_SET_OFF_TORQUE_ROD_3,				// 10
	ADCS_CMD_SET_POLARITY_TORQUE_ROD_1,			// 11
	ADCS_CMD_SET_POLARITY_TORQUE_ROD_2,			// 12
	ADCS_CMD_SET_POLARITY_TORQUE_ROD_3,			// 13
	ADCS_CMD_SET_PWM_COUNTER_TORQUE_ROD,		// 14
	ADCS_CMD_GET_MEASUREMENT_SUN_SENSOR,		// 15
	ADCS_CMD_GET_MEASUREMENT_GYRO_1,			// 16
	ADCS_CMD_GET_MEASUREMENT_GYRO_2,			// 17
	ADCS_GYRO_ACK,
	ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_1,	// 18
	ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_2,	// 19
	ADCS_MAGNETO_ACK,
	ADCS_SYNC_SPI,								// 20
	NUM_ADCS_COMMANDS,
	ADCS_ACK=55,									// 21
	ADCS_SPI_CMD_ERROR = 75,
} AdcsCommand_t;

/*
 * Utilities
 * @return
 */
AdcsDriverError_t adcs_init_driver(void)
{
	AdcsDriverError_t status = ADCS_DRIVER_NO_ERROR;
}

AdcsDriverError_t adcsTxRx(uint8_t * tx_data, uint16_t tx_size, uint8_t * rx_data, uint16_t rx_size)
{
	spi_transaction_block_read_without_toggle(
			ADCS_SPI_CORE,
			ADCS_SLAVE_CORE,
			tx_data,
			tx_size,
			rx_data,
			rx_size);
	return ADCS_DRIVER_NO_ERROR;
}

/*
 * ADCS Utility Commands
 * @return
 */
AdcsDriverError_t pingAdcs(void)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
	uint8_t cmd_id = ADCS_CMD_PING;
	uint8_t cmd_ack = 0;
	error_code = adcsTxRx(&cmd_id,1,&cmd_ack,1);

	return error_code;
}

AdcsDriverError_t adcsSyncSpi(void)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
	uint8_t cmd_id = ADCS_CMD_PING;
	uint8_t cmd_ack = 0;
	do
	{
		AdcsDriverError_t error_code = adcsTxRx(&cmd_id,1,&cmd_ack,1);
	} while(cmd_ack != ADCS_SYNC_SPI);

	return error_code;
}
/*
 * ADCS Torque Rod Commands
 * @return
 */

AdcsDriverError_t setTorqueRodState(TortqueRodId_t rod_number, uint8_t rod_state)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ID;
	uint8_t cmd_id = -1;
	// Get Command ID
	switch(rod_state)
	{
	case 0:
	default:
		return error_code;
	}
	switch(rod_number)
	{
	case TORQUE_ROD_1:
		cmd_id = ADCS_CMD_SET_ON_TORQUE_ROD_1;
		break;
	case TORQUE_ROD_2:
		cmd_id = ADCS_CMD_SET_ON_TORQUE_ROD_2;
		break;
	case TORQUE_ROD_3:
		cmd_id = ADCS_CMD_SET_ON_TORQUE_ROD_3;
		break;
	default:
		return error_code;
	}
	// SPI transactions
	uint8_t cmd_ack = 0;
	error_code = adcsTxRx(&cmd_id,1,&cmd_ack,1);

	return error_code;
}

AdcsDriverError_t setTorqueRodPolarity(TortqueRodId_t rod_number, uint8_t polarity)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ID;
	uint8_t cmd_id = -1;
	// Get Command ID
	switch(rod_number)
	{
	case TORQUE_ROD_1:
		cmd_id = ADCS_CMD_SET_POLARITY_TORQUE_ROD_1;
		break;
	case TORQUE_ROD_2:
		cmd_id = ADCS_CMD_SET_POLARITY_TORQUE_ROD_2;
		break;
	case TORQUE_ROD_3:
		cmd_id = ADCS_CMD_SET_POLARITY_TORQUE_ROD_3;
		break;
	default:
		return error_code;
	}
	// SPI transactions
	uint8_t tx_data[2] = {cmd_id,polarity};
	uint8_t cmd_ack = 0;
	error_code = adcsTxRx(&tx_data,2,&cmd_ack,1);

	return error_code;
}

AdcsDriverError_t setTorqueRodPwm(TortqueRodId_t rod_number, uint8_t pwm)
{
	AdcsDriverError_t error_code = ADCS_ERROR_BAD_ID;
	uint8_t cmd_id = -1;
	// Get Command ID
	switch(rod_number)
	{
	case TORQUE_ROD_1:
		cmd_id = ADCS_CMD_SET_PWM_TORQUE_ROD_1;
		break;
	case TORQUE_ROD_2:
		cmd_id = ADCS_CMD_SET_PWM_TORQUE_ROD_2;
		break;
	case TORQUE_ROD_3:
		cmd_id = ADCS_CMD_SET_PWM_TORQUE_ROD_3;
		break;
	default:
		return error_code;
	}
	// SPI transactions
	uint8_t tx_data[2] = {cmd_id,pwm};
	uint8_t cmd_ack = 0;
	error_code = adcsTxRx(&tx_data,2,&cmd_ack,1);

	return error_code;
}

/*
 * ADCS Sensor Polling Commands
 * @return
 */

AdcsDriverError_t getGyroMeasurements(GyroId_t gyroNumber, uint8_t * gyroMeasurements)
{
    // Get command ID
    uint8_t cmd_id = -1;
    switch(gyroNumber)
    {
    case GYRO_1:
        cmd_id = ADCS_CMD_GET_MEASUREMENT_GYRO_1;
        break;
    case GYRO_2:
        cmd_id = ADCS_CMD_GET_MEASUREMENT_GYRO_2;
        break;
    default:
        return ADCS_ERROR_BAD_ID;
    }
    // SPI transactions
    AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
    // Command ack
	uint8_t cmd_ack = 0;
	error_code = adcsTxRx(&cmd_id,1,&cmd_ack,1);
    // Get measurements
	uint8_t tx_data[6] = {0};
	error_code = adcsTxRx(&tx_data,ADCS_GYRO_DATA_SIZE,&gyroMeasurements,ADCS_GYRO_DATA_SIZE);

	return error_code;
}

AdcsDriverError_t getMagnetometerMeasurements(MagnetometerId_t magnetometerNumber, uint8_t * magnetometerMeasurements)
{
    // Get command ID
    uint8_t cmd_id = -1;
    switch(magnetometerNumber)
    {
    case GYRO_1:
        cmd_id = ADCS_CMD_GET_MEASUREMENT_GYRO_1;
        break;
    case GYRO_2:
        cmd_id = ADCS_CMD_GET_MEASUREMENT_GYRO_2;
        break;
    default:
        return ADCS_ERROR_BAD_ID;
    }
    // SPI transactions
    AdcsDriverError_t error_code = ADCS_ERROR_BAD_ACK;
    // Command ack
	uint8_t cmd_ack = 0;
	error_code = adcsTxRx(&cmd_id,1,&cmd_ack,1);
    // Get measurements
	uint8_t tx_data[6] = {0};
	error_code = adcsTxRx(&tx_data,ADCS_MAGNETORQUER_DATA_SIZE,&magnetometerMeasurements,ADCS_MAGNETORQUER_DATA_SIZE);

	return error_code;
}

AdcsDriverError_t getSunSensorMeasurements(uint8_t * measurements)
{
	uint8_t cmd_id = ADCS_CMD_GET_MEASUREMENT_SUN_SENSOR;
	AdcsDriverError_t error_code = adcsTxRx(&cmd_id,1,&measurements,ADCS_SUN_SENSOR_DATA_SIZE);
}



