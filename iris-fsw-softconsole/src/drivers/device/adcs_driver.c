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

#define SPI_EFFICIENT

#define ADCS_ACK_PREFIX 0x01
#define MAX_SYNC_CYCLES 10

// ADCS Command IDs
typedef enum
{
	ADCS_CMD_PING = 1,
	ADCS_CMD_SET_PWM_TORQUE_ROD_1,						// 2
	ADCS_CMD_SET_PWM_TORQUE_ROD_2,						// 3
	ADCS_CMD_SET_PWM_TORQUE_ROD_3,						// 4
	ADCS_CMD_SET_ON_TORQUE_ROD_1,							// 5
	ADCS_CMD_SET_ON_TORQUE_ROD_2,							// 6
	ADCS_CMD_SET_ON_TORQUE_ROD_3,							// 7
	ADCS_CMD_SET_OFF_TORQUE_ROD_1,						// 8
	ADCS_CMD_SET_OFF_TORQUE_ROD_2,						// 9
	ADCS_CMD_SET_OFF_TORQUE_ROD_3,						// 10
	ADCS_CMD_SET_POLARITY_TORQUE_ROD_1,					// 11
	ADCS_CMD_SET_POLARITY_TORQUE_ROD_2,					// 12
	ADCS_CMD_SET_POLARITY_TORQUE_ROD_3,					// 13
	ADCS_CMD_SET_PWM_COUNTER_TORQUE_ROD,				// 14
	ADCS_CMD_GET_MEASUREMENT_SUN_SENSOR_PRIMARY,		// 15
	ADCS_CMD_GET_MEASUREMENT_GYRO_1,						// 16
	ADCS_CMD_GET_MEASUREMENT_GYRO_2,						// 17
	ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_1,			// 18
	ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_2,			// 19
	ADCS_SPI_PORT_TOGGLE,									// 20
	ADCS_SYNC_SPI,												// 21
	NUM_ADCS_COMMANDS,
	ADCS_ACK=55,
	ADCS_SPI_CMD_ERROR = 75,
} AdcsCommands_t;

/*
 * Utilities
 * @return
 */
AdcsDriverError_t adcs_init_driver(void)
{
	AdcsDriverError_t status = ADCS_DRIVER_NO_ERROR;
	return status;
}

#ifdef SPI_EFFICIENT
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
#elif
AdcsDriverError_t adcsTxRx(uint8_t * tx_data, uint16_t tx_size, uint8_t * rx_data, uint16_t rx_size)
{
	uint8_t i;
	for(i=0; i < tx_size; i++)
	{
		spi_transaction_block_read_without_toggle(
				ADCS_SPI_CORE,
				ADCS_SLAVE_CORE,
				&tx_data[i],
				1,
				NULL,
				0);
	}
	for(i=0; i < rx_size; i++)
	{
		spi_transaction_block_read_without_toggle(
				ADCS_SPI_CORE,
				ADCS_SLAVE_CORE,
				NULL,
				0,
				&rx_data[i],
				1);
	}
}
#endif

/*
 * ADCS Utility Commands
 * @return
 */
AdcsDriverError_t pingAdcs(void)
{
	AdcsDriverError_t status = ADCS_ERROR_BAD_ACK;
	status = adcsSyncSpiCommand(ADCS_CMD_PING);
	return status;
}

AdcsDriverError_t adcsSyncSpiCommand(uint8_t cmd_id)
{
    AdcsDriverError_t status = ADCS_ERROR_BAD_ACK;
    // Command ack
	uint8_t cmd_ack = 0;
//	uint8_t cmd[2] = {cmd_id,cmd_id};
//	status = adcsTxRx(cmd,2,NULL,0);
	status = adcsTxRx(&cmd_id,1,NULL,0);
	vTaskDelay(10);
	status = adcsTxRx(NULL,0,&cmd_ack,1);
	// Verify result
	if(cmd_id == cmd_ack)
	{
		status = ADCS_DRIVER_NO_ERROR;
	}
	else
	{
		status = ADCS_ERROR_BAD_ACK;
	}
	return status;
}

AdcsDriverError_t adcsSyncSpi(void)
{
	AdcsDriverError_t status = ADCS_ERROR_BAD_ACK;
	uint8_t cmd_id = ADCS_CMD_PING;
	uint8_t cmd_ack = 0;
	uint8_t cycles = 0;
	while(cmd_ack != ADCS_SYNC_SPI && cycles != MAX_SYNC_CYCLES)
	{
		AdcsDriverError_t status = adcsTxRx(&cmd_id,1,&cmd_ack,0);
		cycles++;
	}

	if(cycles == MAX_SYNC_CYCLES)
	{
		status = ADCS_ERROR_BAD_ID;
	}

	return status;
}
/*
 * ADCS Torque Rod Commands
 * @return
 */

AdcsDriverError_t setTorqueRodState(TorqueRodId_t rod_number, TortqueRodState_t rod_state)
{
	AdcsDriverError_t status = ADCS_ERROR_BAD_ID;
	uint8_t cmd_id = -1;
	// Get Command ID
	switch(rod_state)
	{
		case TR_STATE_OFF:{
			switch(rod_number)
			{
			case TORQUE_ROD_1:
				cmd_id = ADCS_CMD_SET_OFF_TORQUE_ROD_1;
				break;
			case TORQUE_ROD_2:
				cmd_id = ADCS_CMD_SET_OFF_TORQUE_ROD_2;
				break;
			case TORQUE_ROD_3:
				cmd_id = ADCS_CMD_SET_OFF_TORQUE_ROD_3;
				break;
			default:
				return status;
			}
			break;
		}
		case TR_STATE_ON:{
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
				return status;
			}
			break;
		}
		default:{
			return status;
		}
	} // End of switch(rod_state)
	// Issue command
	status = adcsSyncSpiCommand(cmd_id);

	return status;
}

AdcsDriverError_t setTorqueRodPolarity(TorqueRodId_t rod_number, uint8_t polarity)
{
	AdcsDriverError_t status = ADCS_ERROR_BAD_ID;
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
		return status;
	}
	// Issue command
	status = adcsSyncSpiCommand(cmd_id);
	// Set polarity
	status = adcsTxRx(&polarity,1,NULL,0);

	return status;
}

AdcsDriverError_t setTorqueRodPwm(TorqueRodId_t rod_number, uint8_t pwm)
{
	AdcsDriverError_t status = ADCS_ERROR_BAD_ID;
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
		return status;
	}
	// Issue command
	status = adcsSyncSpiCommand(cmd_id);
	// Set PWM
	status = adcsTxRx(&pwm,1,NULL,0);

	return status;
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
    AdcsDriverError_t status = ADCS_ERROR_BAD_ACK;
    // Command ack
    status = adcsSyncSpiCommand(cmd_id);
	vTaskDelay(100);
    // Get measurements
	status = adcsTxRx(NULL,0,gyroMeasurements,ADCS_GYRO_DATA_SIZE);

	return status;
}

AdcsDriverError_t getMagnetometerMeasurements(MagnetometerId_t magnetometerNumber, uint8_t * magnetometerMeasurements)
{
    // Get command ID
    uint8_t cmd_id = -1;
    switch(magnetometerNumber)
    {
    case GYRO_1:
        cmd_id = ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_1;
        break;
    case GYRO_2:
        cmd_id = ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_2;
        break;
    default:
        return ADCS_ERROR_BAD_ID;
    }
    // SPI transactions
    AdcsDriverError_t status = ADCS_ERROR_BAD_ACK;
    // Command ack
    status = adcsSyncSpiCommand(cmd_id);
	vTaskDelay(10);
    // Get measurements
	status = adcsTxRx(NULL,0,magnetometerMeasurements,ADCS_MAGNETORQUER_DATA_SIZE);

	return status;
}

AdcsDriverError_t getSunSensorMeasurements(uint8_t * measurements)
{
	uint8_t cmd_id = ADCS_CMD_GET_MEASUREMENT_SUN_SENSOR_PRIMARY;
	AdcsDriverError_t status;
	status = adcsSyncSpiCommand(cmd_id);
	status = adcsTxRx(NULL,0,&measurements[0],ADCS_SUN_SENSOR_DATA_SIZE);
	status = adcsTxRx(NULL,0,&measurements[164],ADCS_SUN_SENSOR_DATA_SIZE);
	return status;
}



