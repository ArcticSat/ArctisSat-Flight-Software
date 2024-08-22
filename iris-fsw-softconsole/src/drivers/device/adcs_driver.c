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

//#define SPI_EFFICIENT

#define ADCS_ACK_PREFIX 0x01
#define MAX_SYNC_CYCLES 180

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
	ADCS_SELECT_SS1,											// 15
	ADCS_SELECT_SS2,											// 16
	ADCS_SELECT_SS3,											// 17
	ADCS_SELECT_SS4,											// 18
	ADCS_CMD_GET_MEASUREMENT_SUN_SENSOR,				// 19
	ADCS_SET_GYRO_I2C_ADDRESS,								// 20
	ADCS_GET_GYRO_MEASUREMENT,								// 21
	ADCS_CMD_GET_MEASUREMENT_GYRO_1,						// 22
	ADCS_CMD_GET_MEASUREMENT_GYRO_2,						// 23
	ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_1,			// 24
	ADCS_CMD_GET_MEASUREMENT_MAGNETOMETER_2,			// 25
	ADCS_SPI_PORT_TOGGLE,									// 26
	ADCS_SYNC_SPI,												// 27
	NUM_ADCS_COMMANDS,										// 28
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
#else
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
	uint8_t cmd_id = 0xFF;
	uint8_t cmd_ack = 0;
	uint8_t cycles = 0;
	while(cmd_ack != 0xFF && cycles != MAX_SYNC_CYCLES)
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

AdcsDriverError_t setGyroI2cAddress(uint8_t addr)
{
    uint8_t cmd_id = ADCS_SET_GYRO_I2C_ADDRESS;
    AdcsDriverError_t status = ADCS_ERROR_BAD_ACK;
    // SPI transactions
    status = adcsSyncSpiCommand(cmd_id); // Command ID
    vTaskDelay(10);
	status = adcsTxRx(NULL,0,&addr,1); // Address

    return status;
}

AdcsDriverError_t getGyroMeasurementsGenericRaw(uint8_t * gyroMeasurements)
{
    uint8_t cmd_id = ADCS_GET_GYRO_MEASUREMENT;
    AdcsDriverError_t status = ADCS_ERROR_BAD_ACK;
    // SPI transactions
    status = adcsSyncSpiCommand(cmd_id);
	vTaskDelay(100);
    // Get measurements
	status = adcsTxRx(NULL,0,gyroMeasurements,ADCS_GYRO_RAW_DATA_SIZE_BYTES);

	return status;
}

AdcsDriverError_t getGyroMeasurementsRaw(GyroId_t gyroNumber, uint8_t * gyroMeasurements)
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
	status = adcsTxRx(NULL,0,gyroMeasurements,ADCS_GYRO_RAW_DATA_SIZE_BYTES);

	return status;
}

AdcsDriverError_t getMagnetometerMeasurementsRaw(MagnetometerId_t magnetometerNumber, uint8_t * magnetometerMeasurements)
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
	status = adcsTxRx(NULL,0,magnetometerMeasurements,ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES);

	return status;
}


AdcsDriverError_t sunSensorSelect(enumSunSensor sunSensor)
{
	uint8_t cmd_id;
	AdcsDriverError_t status;
	switch(sunSensor)
	{
		case SUN_SENSOR_1:
			cmd_id = ADCS_SELECT_SS1;
			break;
		case SUN_SENSOR_2:
			cmd_id = ADCS_SELECT_SS2;
			break;
		case SUN_SENSOR_3:
			cmd_id = ADCS_SELECT_SS3;
			break;
		case SUN_SENSOR_4:
			cmd_id = ADCS_SELECT_SS4;
			break;
		default:
			return ADCS_ERROR_BAD_ID;
	}
	status = adcsSyncSpiCommand(cmd_id);
	return status;
}

AdcsDriverError_t getSunSensorMeasurementsRaw(uint8_t * measurements)
{
	uint8_t cmd_id = ADCS_CMD_GET_MEASUREMENT_SUN_SENSOR;
	AdcsDriverError_t status;
	status = adcsSyncSpiCommand(cmd_id);
	vTaskDelay(20);
	status = adcsTxRx(NULL,0,&measurements[0],ADCS_SUN_SENSOR_DATA_SIZE);
//	vTaskDelay(20);
//	status = adcsTxRx(NULL,0,&measurements[164],ADCS_SUN_SENSOR_DATA_SIZE);
	return status;
}

/*** Raw sensor data conversion ***/
// Gyro
float a3g4250d_from_fs245dps_to_mdps(int16_t lsb)
{
	return ((float)lsb * 8.75f);
}

// Mag
float mmc5883ma_from_fs8G_to_mG(uint16_t mag_fs_raw)
{
	uint16_t full_scale_normalized = mag_fs_raw - UINT16_MAX / 2;
//	float magnetic_field = ((float)full_scale_normalized) * A3G4250D_FULL_SCALE_MAX;
//	return magnetic_field
	return 0.0;
}

// Sun
uint16_t AngleDecompose(uint8_t *RXBuff,uint8_t selec )
{
    uint8_t FirstPxlPosition=0,i=0,j=0,ROMask[4] = {0x80,0x80,0xF0,0xFF},ROShift[4] = {1,2,4,8},offset[3] = {1,0,0},RODiv[4] = {1,2,15,255},rxSize[4] = {18,36,72,144},Temp = ROMask[selec];
    float FirstPxlFrac=0;
    uint16_t AngFrac=0;

    while(RXBuff[i] == 0)       //Skips the unilluminated pixels
    {
        i++;
    }
    while(!(RXBuff[i] & Temp))  //Honestly don't remember why I had this
    {
        Temp = Temp >> ROShift[selec];
        j++;
        if(j>=ROShift[3-selec])
            break;
    }
    switch (selec)      // This depends on the readout mode which is specified with "selec". The code does some binary operations which I do not recall how I wrote, but it should work.
    {                   // As far as I recall, this code essentially breaks down the gathered raw data to find the first pixel with light, and figures out the fraction.
        case 0:
            FirstPxlPosition = (i*ROShift[3-selec]) + j+1;      // FirstPxlPosition is the position of the first fully illuminated pixel
            FirstPxlFrac = 0;                                   // Fraction of the partially illuminated pixel which should be before the fully illuminated pixel. These then get added up in the final step
            break;
        case 1:
            FirstPxlPosition = (i*ROShift[3-selec]) + j+1;
            FirstPxlFrac= (float)((RXBuff[i] & (0xC0>>(ROShift[selec]*(j-1)))) >> ((4-j)*ROShift[selec]))/(float)RODiv[selec];
            break;
        case 2:
            FirstPxlPosition = (i*ROShift[3-selec]) + j+2;
            FirstPxlFrac= (float)((RXBuff[i] & (0xF0>>(ROShift[selec]*j))) >> ((1-j)*ROShift[selec]))/(float)RODiv[selec];
            break;
        case 3:
            FirstPxlPosition = (i*ROShift[3-selec]) + j+2;
            FirstPxlFrac = (float)RXBuff[i]/(float)RODiv[selec];
            break;
    }
    AngFrac = 1000*(90-(atan(MASK_HEIGHT/((REF_PIXEL-(FirstPxlPosition-FirstPxlFrac))*PIXEL_LENGTH))*180/M_PI)); // This step uses simple trigonometry to calculate the angle using the incidence length.
    return AngFrac;
}

/*** Application-level sensor polling ***/
// Gyro
AdcsDriverError_t getGyroscopeDataRadiansPerSecond(GyroId_t gyroNumber, float * gyroDataRps)
{
	AdcsDriverError_t status = ADCS_DRIVER_NO_ERROR;
	uint8_t gyroRaw8[ADCS_GYRO_RAW_DATA_SIZE_BYTES] = {0};
	// Get raw data
	status = getGyroMeasurementsRaw(gyroNumber,gyroRaw8);
	if(status != ADCS_DRIVER_NO_ERROR) return status;
	// Convert raw data
	int i;
	for(i=0; i < 3; i++){
		// Format raw axis data as 16-bit unsigned integer
		uint16_t gyroRaw16 = 0;
		gyroRaw16 |= (uint16_t) gyroRaw8[2*i];				// LSB transferred first
		gyroRaw16 |= (((uint16_t) gyroRaw8[2*i+1]) << 8);	// MSB second
		// Convert raw sample to radians/s
		gyroDataRps[i] = ((int16_t) gyroRaw16) * DPS_TO_RPS;
	}

	return status;
}
// Mag
AdcsDriverError_t getMagnetometerDataTeslas(MagnetometerId_t magnetometerNumber, float * magDataTeslas)
{
	AdcsDriverError_t status = ADCS_DRIVER_NO_ERROR;
	uint8_t magRaw8[ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES] = {0};
	// Get raw data
	status = getMagnetometerMeasurementsRaw(magnetometerNumber,magRaw8);
	if(status != ADCS_DRIVER_NO_ERROR) return status;
	// Convert raw data
	int i;
	for(i=0; i < 3; i++){
		// Format raw axis data as 16-bit unsigned integer
		uint16_t magRaw16 = 0;
		magRaw16 |= (uint16_t) magRaw8[2*i];				// LSB transferred first
		magRaw16 |= (((uint16_t) magRaw8[2*i+1]) << 8);
		// Convert raw sample to Teslas
//		magDataTeslas[i] = ((-8) + (magRaw16 * MAG_LSB)) * MILLIGAUSS_TO_TESLA_CONVERSION;
	}

	return status;
}
// Sun
AdcsDriverError_t getSunAngle(uint8_t * measurements)
{
	AdcsDriverError_t status = ADCS_DRIVER_NO_ERROR;

	return status;
}

