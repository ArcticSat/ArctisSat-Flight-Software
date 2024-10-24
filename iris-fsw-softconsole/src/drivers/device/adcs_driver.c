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

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "drivers/device/adcs_driver.h"
#include "drivers/subsystems/eps_driver.h"
#include "board_definitions.h"

//#define SPI_EFFICIENT

#define ADCS_ACK_PREFIX 0x01
#define MAX_SYNC_CYCLES 180
// Magnetometer calibration parameters
#define MAGNETOMETER_CALIBRATION_SAMPLES	5
// Sun sensor parameters
#define SUN_MAX_DETECTABLE_ANGLE 50.0
// Orbit time
#define SPACECRAFT_ORBIT_TIME_HOURS 	1
#define SPACECRAFT_ORBIT_TIME_MINUTES 	32
#define SPACECRAFT_ORBIT_TIME_SECONDS 	18
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Sun sensor conversion variables
const uint8_t ROMask[4] = {0x80,0x80,0xF0,0xFF};
const uint8_t ROShift[4] = {1,2,4,8};
const uint8_t sunDataOffset[3] = {1,0,0};
const uint8_t RODiv[4] = {1,2,15,255};
const uint8_t sunDataRxSize[4] = {18,36,72,144};
// Magnetometer calibration variables
float mag_sign_flip[3] = {1.0,-1.0,1.0};
float mag_offsets[3] = {0.0,0.0,0.0};
float mag_x_offset = 0.0;
float mag_y_offset = 0.0;
float mag_z_offset = 0.0;
// Sun vector conversion
volatile float ss_x_sign_flip =  1.0;
volatile float ss_z_sign_flip = -1.0;
// Eclipse variables
volatile Calendar_t eclipseBounds[2] = {0xFF};
// Backwards variables
volatile double sun_angle_max_vector_length = sin(SUN_MAX_DETECTABLE_ANGLE);
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


void vTestAdcsDriverInterface(void * pvParameters)
{
	int i;
	telemetryPacket_t tmpkt = {0};
	uint8_t buf[20];
	uint8_t driver_status;
	uint8_t gyro_id = GYRO_1;
	MagnetometerId_t mag_id = MAGNETOMETER_1;
	uint8_t gyro_buf[6] = {0};
	uint16_t gyro_raw16[3] = {0};
	int16_t  gyro_raw_int16[3] = {0};
	short  gyro_raw_short[3] = {0};
	uint8_t mag_buf[6] = {0};
	float float_buf[3];
	float gyro_data[3] = {0};
	float mag_data[3] = {0};

	uint8_t sun_buf[164];
	uint8_t x_sun_buf[164];
	uint8_t z_sun_buf[164];
	float sunAngle = 0.0;
	float z_angle,x_angle;

	setLoadSwitch(LS_ADCS,SWITCH_OFF);
	vTaskDelay(1000);
	setLoadSwitch(LS_ADCS,SWITCH_ON);
	vTaskDelay(1000);
	CalibrateMagnetometerSingleTorqueRod(MAGNETOMETER_1);

	while(1)
	{
//		/*** Gyro data ***/
//		memcpy(gyro_buf,0,6*sizeof(uint8_t));
//		// Get raw data
//		driver_status = (uint8_t) getGyroMeasurementsRaw(gyro_id, gyro_buf);
//		// Convert
//		memcpy(gyro_data,0,3*sizeof(float));
//		for(i=0; i < 3; i++){
//			uint16_t gyro_raw;
//			gyro_raw = ((uint16_t)  gyro_buf[2*i]);
//			gyro_raw |= (((uint16_t) gyro_buf[2*i+1]) << 8);
//			float_buf[i] = convertGyroDataRawToRadiansPerSecond(gyro_raw);
//			gyro_raw16[i] = gyro_raw;
//			gyro_raw_int16[i] = (int16_t) gyro_raw16[i];
//			gyro_raw_short[i] = (short) gyro_raw16[i];
//		}
//		// Orient
//		if(gyro_id == GYRO_1){
//			gyro_data[0] = -float_buf[1]; // X <-- (-Y)
//			gyro_data[1] =  float_buf[0]; // Y <--   X
//			gyro_data[2] =  float_buf[2]; // Z <--   Z
//		} else if(gyro_id == GYRO_2) {
//			gyro_data[0] =  float_buf[0]; // X <--   X
//			gyro_data[1] = -float_buf[1]; // Y <-- (-Y)
//			gyro_data[2] =  float_buf[2]; // Z <--   Z
//		}
//		// Send telemetry
//		memcpy(buf,0,20);
//		tmpkt.telem_id = ADCS_GYROSCOPE_DATA_ID;
//		tmpkt.length = 14;
//		buf[0] = driver_status;
//		buf[1] = gyro_id;
//		memcpy(&buf[2],gyro_data,3*sizeof(float));
//		tmpkt.data = buf;
//		sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
//		int x = 7;
//		vTaskDelay(10);

//		/*** Mag data ***/
//		// Get raw data
//		// Convert
//		memcpy(mag_buf,0,3*sizeof(float));
//		driver_status = (uint8_t) getMagnetometerMeasurementsRaw(mag_id, mag_buf);
//		for(i=0; i < 3; i++){
//			uint16_t mag_raw = 0;
//			mag_raw |= ((uint16_t)  mag_buf[2*i]);
//			mag_raw |= (((uint16_t) mag_buf[2*i+1]) << 8);
//			float_buf[i] = convertMagDataRawToTeslas(mag_raw);
//		}
//		// Orient, offset
//		mag_data[0] =  float_buf[0]; // X <--   X
//		mag_data[1] = -float_buf[1]; // Y <-- (-Y)
//		mag_data[2] =  float_buf[2]; // Z <--   Z
//		for(i=0; i < 3; i++)
//		{
//			mag_data[i] -= mag_offsets[i];
//			mag_data[i] *= 1000000.0;
//		}
//		// Send telemetry
//		memcpy(buf,0,20);
//		tmpkt.telem_id = ADCS_MAGNETOMETER_DATA_ID;
//		tmpkt.length = 14;
//		buf[0] = driver_status;
//		buf[1] = mag_id;
//		memcpy(&buf[2],mag_data,3*sizeof(float));
//		tmpkt.data = buf;
//		sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
//		int j = 7;

//		vTaskDelay(10);

		/*** Sun data ***/
//		driver_status = sunSensorSelect(SUN_SENSOR_1);
//		vTaskDelay(10);
//		driver_status = getSunSensorMeasurementsRaw(sun_buf);
//		// Send meta data
//		tmpkt.telem_id = ADCS_SS_META_ID;
//		tmpkt.length = 12;
//		tmpkt.data = buf;
//		buf[0] = (uint8_t) driver_status;
//		memcpy(&buf[1],sun_buf,12);
//		sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
//		vTaskDelay(10);
//		// Send raw data
//		tmpkt.telem_id = ADCS_SS_RAW_ID;
//		tmpkt.length = 20;
//		for(i=0; i < 124; i+=20){
//			tmpkt.data = &sun_buf[i];
//			sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
//			vTaskDelay(10);
//		}
//		// Send angle
//		sunAngle = AngleDecompose(sun_buf,3);
//		tmpkt.telem_id = ADCS_SS_ANGLE_ID;
//		tmpkt.length = 4;
//		tmpkt.data = buf;
//		memcpy(buf,&sunAngle,sizeof(sunAngle));
//		sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
//		int x = 7;
//		vTaskDelay(10);

		// Unit vector
		driver_status = sunSensorSelect(SUN_SENSOR_PRIMARY_Z);
		vTaskDelay(10);
		driver_status = getSunSensorMeasurementsRaw(z_sun_buf);
		vTaskDelay(10);
		z_angle = AngleDecompose(z_sun_buf,3);
//		z_angle -= z_angle;
		z_angle *= ss_z_sign_flip;
		driver_status = sunSensorSelect(SUN_SENSOR_PRIMARY_X);
		vTaskDelay(10);
		driver_status = getSunSensorMeasurementsRaw(x_sun_buf);
		vTaskDelay(10);
		x_angle = AngleDecompose(x_sun_buf,3);
		x_angle *= ss_x_sign_flip;

		float unit_vector[2] = {0.0};
		unit_vector[0] = sin(x_angle);
		unit_vector[1] = sin(z_angle);

		tmpkt.telem_id = ADCS_SS_UNIT_VECTOR_ID;
		tmpkt.length = 8;
		tmpkt.data = buf;
		memcpy(buf,&unit_vector,sizeof(unit_vector));
		sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);

		vTaskDelay(2000);

	}
}

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
#if defined(FLIGHT_MODEL_CONFIGURATION) || defined(ENGINEERING_MODEL_CONFIGURATION)
	spi_transaction_block_read_without_toggle(
			ADCS_SPI_CORE,
			ADCS_SLAVE_CORE,
			tx_data,
			tx_size,
			rx_data,
			rx_size);
	return ADCS_DRIVER_NO_ERROR;
#elif defined(MAKER2_DEVKIT_CONFIGURATION)
	spi_transaction_block_read_without_toggle(
			FLASH_SPI_CORE,
			ADCS_SLAVE_CORE,
			tx_data,
			tx_size,
			rx_data,
			rx_size);
	return ADCS_DRIVER_NO_ERROR;
#endif
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
	return ADCS_DRIVER_NO_ERROR;
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

AdcsDriverError_t setTorqueRodState(MagnetorquerID_t rod_number, TortqueRodState_t rod_state)
{
	AdcsDriverError_t status = ADCS_ERROR_BAD_ID;
	uint8_t cmd_id = -1;
	// Get Command ID
	switch(rod_state)
	{
		case TR_STATE_OFF:{
			switch(rod_number)
			{
			case MAGNETORQUER_X:
				cmd_id = ADCS_CMD_SET_OFF_TORQUE_ROD_2;
				break;
			case MAGNETORQUER_Y:
				cmd_id = ADCS_CMD_SET_OFF_TORQUE_ROD_1;
				break;
			case MAGNETORQUER_Z:
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
			case MAGNETORQUER_X:
				cmd_id = ADCS_CMD_SET_ON_TORQUE_ROD_2;
				break;
			case MAGNETORQUER_Y:
				cmd_id = ADCS_CMD_SET_ON_TORQUE_ROD_1;
				break;
			case MAGNETORQUER_Z:
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

AdcsDriverError_t setTorqueRodPolarity(MagnetorquerID_t rod_number, uint8_t polarity)
{
#ifdef TORQUE_ROD_SIM
	return ADCS_DRIVER_NO_ERROR;
#else
	AdcsDriverError_t status = ADCS_ERROR_BAD_ID;
	uint8_t cmd_id = -1;
	// Get Command ID
	switch(rod_number)
	{
	case MAGNETORQUER_X:
		cmd_id = ADCS_CMD_SET_POLARITY_TORQUE_ROD_2;
		break;
	case MAGNETORQUER_Y:
		cmd_id = ADCS_CMD_SET_POLARITY_TORQUE_ROD_1;
		if(polarity == TR_POLARITY_NEG)
			polarity = TR_POLARITY_POS;
		else if(polarity == TR_POLARITY_POS)
			polarity = TR_POLARITY_NEG;
		break;
	case MAGNETORQUER_Z:
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
#endif
}

AdcsDriverError_t setTorqueRodPwm(MagnetorquerID_t rod_number, uint8_t pwm)
{
#ifdef TORQUE_ROD_SIM
	return ADCS_DRIVER_NO_ERROR;
#else
	AdcsDriverError_t status = ADCS_ERROR_BAD_ID;
	uint8_t cmd_id = -1;
	// Get Command ID
	switch(rod_number)
	{
	case MAGNETORQUER_X:
		cmd_id = ADCS_CMD_SET_PWM_TORQUE_ROD_2;
		break;
	case MAGNETORQUER_Y:
		cmd_id = ADCS_CMD_SET_PWM_TORQUE_ROD_1;
		break;
	case MAGNETORQUER_Z:
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
#endif
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
	// Check valid data
	int i;
	uint32_t data_sum = 0;
	for(i=0; i < ADCS_GYRO_RAW_DATA_SIZE_BYTES; i++)
	{
		data_sum += (uint32_t) gyroMeasurements[i];
	}
	if(data_sum == 0)
	{
		return ADCS_ERROR_BAD_DATA;
	}
	return status;
}

AdcsDriverError_t getGyroMeasurementsRaw(GyroId_t gyroNumber, uint8_t * gyroMeasurements)
{
#ifdef GYRO_SIM_ENG_VALUE
	return ADCS_DRIVER_NO_ERROR;
#else
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
    if(status != ADCS_DRIVER_NO_ERROR)
    	return status;
	vTaskDelay(10);
    // Get measurements
	status = adcsTxRx(NULL,0,gyroMeasurements,ADCS_GYRO_RAW_DATA_SIZE_BYTES);
	// Check valid data
	int i;
	uint32_t data_sum = 0;
	for(i=0; i < ADCS_GYRO_RAW_DATA_SIZE_BYTES; i++)
	{
		data_sum += (uint32_t) gyroMeasurements[i];
	}
	if(data_sum == 0)
	{
		return ADCS_ERROR_BAD_DATA;
	}

	return status;
#endif
}

AdcsDriverError_t getMagnetometerMeasurementsRaw(MagnetometerId_t magnetometerNumber, uint8_t * magnetometerMeasurements)
{
#ifdef MAG_SIM_ENG_VALUE
	return ADCS_DRIVER_NO_ERROR;
#else
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
	// Check valid data
	int i;
	uint32_t data_sum = 0;
	for(i=0; i < ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES; i++)
	{
		data_sum += (uint32_t) magnetometerMeasurements[i];
	}
	if(data_sum == 0)
	{
		return ADCS_ERROR_BAD_DATA;
	}

	return status;
#endif
}


AdcsDriverError_t sunSensorSelect(enumSunSensor sunSensor)
{
#ifdef SUN_ANGLE_SIM_ENG_VALUE
	return ADCS_DRIVER_NO_ERROR;
#else
	uint8_t cmd_id;
	AdcsDriverError_t status;
	switch(sunSensor)
	{
		case SUN_SENSOR_PRIMARY_Z:
			cmd_id = ADCS_SELECT_SS1;
			break;
		case SUN_SENSOR_PRIMARY_X:
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
#endif
}

AdcsDriverError_t getSunSensorMeasurementsRaw(volatile uint8_t * measurements)
{
#ifdef SUN_ANGLE_SIM_ENG_VALUE
	return ADCS_DRIVER_NO_ERROR;
#else
	uint8_t cmd_id = ADCS_CMD_GET_MEASUREMENT_SUN_SENSOR;
	AdcsDriverError_t status;
	status = adcsSyncSpiCommand(cmd_id);
	if(status != ADCS_DRIVER_NO_ERROR)
		return status;
	vTaskDelay(100);
	status = adcsTxRx(NULL,0,&measurements[0],ADCS_SUN_SENSOR_DATA_SIZE);
//	vTaskDelay(20);
//	status = adcsTxRx(NULL,0,&measurements[164],ADCS_SUN_SENSOR_DATA_SIZE);
	return status;
#endif
}
/*** Torque rod data conversion ***/
double dipoleToVoltage(double dipole)
{
	// TODO: check abs()
    if(abs(dipole) > MAX_DIPOLE) {
        return MAX_VOLTAGE * ((dipole >= 0) ? 1 : -1); //convert to proper sign with ternary
    }

    float reqVoltage = 0.0;
    reqVoltage = dipole / DIPOLE_SLOPE;

    if(abs(reqVoltage) > MAX_VOLTAGE) {
        return MAX_VOLTAGE * ((reqVoltage >= 0) ? 1 : -1);
    } else {
        return reqVoltage;
    }
}

void MapTorqueRodCommand(double dipole, uint8_t * polarity, uint8_t * pwm)
{
	double voltage = dipoleToVoltage(dipole);
	if(voltage > 0)
	{
		*polarity = 1;
		*pwm = (uint8_t)(voltage * ( MAX_PWM / MAX_VOLTAGE ));
	}
	else
	{
		*polarity = 0;
		*pwm = (uint8_t)(-voltage * ( MAX_PWM / MAX_VOLTAGE ));
	}
}
/*** Raw sensor data conversion ***/
// Gyro
float convertGyroDataRawToRadiansPerSecond(uint16_t rawGyro)
{
#ifdef GYRO_SIM_ENG_VALUE
	return GYRO_SIM_ENG_VALUE;
#else
	return (float) (( (int16_t) rawGyro * A3G4250D_LSB_DPS) * DPS_TO_RPS );
#endif
}

// Mag
float convertMagDataRawToTeslas(uint16_t rawMag)
{
	uint16_t full_scale_normalized = mag_fs_raw - UINT16_MAX / 2;
//	float magnetic_field = ((float)full_scale_normalized) * A3G4250D_FULL_SCALE_MAX;
//	return magnetic_field
	return 0.0;
}

// Sun
float AngleDecompose(uint8_t *RXBuff,uint8_t selec )
{
#ifdef MAG_SIM_ENG_VALUE
	return MAG_SIM_ENG_VALUE;
#else
	uint8_t FirstPxlPosition=0;
	uint8_t i=sunBufferOffset;
	uint8_t j=0;
//	uint8_t ROMask[4] = {0x80,0x80,0xF0,0xFF};
//	uint8_t ROShift[4] = {1,2,4,8};
//	uint8_t sunDataOffset[3] = {1,0,0};
//	uint8_t RODiv[4] = {1,2,15,255};
//	uint8_t sunDataRxSize[4] = {18,36,72,144};
	uint8_t Temp = ROMask[selec];
	float FirstPxlFrac=0;
	uint16_t AngFrac=0;

	while(RXBuff[i] != 255 && i < sunDataRxSize[selec])       //Skips the unilluminated pixels
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
		FirstPxlPosition = ((i-sunBufferOffset)*ROShift[3-selec]) + j+1;      // FirstPxlPosition is the position of the first fully illuminated pixel
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
		FirstPxlPosition = ((i-sunBufferOffset)*ROShift[3-selec]) + j+2;
		FirstPxlFrac = (float)RXBuff[i]/(float)RODiv[selec];
		break;
	}

	float denominator = ((REF_PIXEL-(FirstPxlPosition-FirstPxlFrac))*PIXEL_LENGTH);
	float sunAngle = 90.0 - atan2(MASK_HEIGHT,denominator)*180.0/M_PI;
//		AngFrac = 1000*(90-(atan(MASK_HEIGHT/denominator)*180/M_PI));
//		 AngFrac = 1000*(90-(atan(MASK_HEIGHT/((REF_PIXEL-(FirstPxlPosition-FirstPxlFrac))*PIXEL_LENGTH))*180/M_PI)); // This step uses simple trigonometry to calculate the angle using the incidence length.
	return sunAngle;
#endif
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
		gyroDataRps[i] = convertGyroDataRawToRadiansPerSecond(gyroRaw16);
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


/*** Sensor calibration ***/
AdcsDriverError_t CalibrateMagnetometer(MagnetometerId_t mag_id)
{
	const TickType_t sample_delay_ms = 10;
	const TickType_t command_delay_ms = 10;
	const TickType_t activation_time_ms = 10;

	uint8_t i;
	uint8_t buf[6];
	AdcsDriverError_t status = ADCS_DRIVER_NO_ERROR;

	float 	 T_avg_pos = {0.0};
	float 	 T_avg_neg = {0.0};

	/*** Power cycle the ADCS ***/
    setLoadSwitch(LS_ADCS,SWITCH_OFF);
    vTaskDelay(100);
    setLoadSwitch(LS_ADCS,SWITCH_ON);
    vTaskDelay(100);

	/*** Turn off all torque rods ***/
	setTorqueRodPwm(MAGNETORQUER_X, 0);
	setTorqueRodPwm(MAGNETORQUER_Y, 0);
	setTorqueRodPwm(MAGNETORQUER_Z, 0);
	vTaskDelay(TORQUE_ROD_DECAY_TIME_MS);

	/*** Calibrate Mag. X ***/
	// Activate +X
	setTorqueRodPolarity(MAGNETORQUER_X, TR_POLARITY_POS);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(MAGNETORQUER_X, 255);
	vTaskDelay(activation_time_ms);
	// Sample Mag. X
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		uint16_t rawMag;
		getMagnetometerMeasurementsRaw(mag_id,buf);
		rawMag = 0;
		rawMag |=  ((uint16_t) buf[0]);
		rawMag |= (((uint16_t) buf[1]) << 8);

		T_avg_pos += convertMagDataRawToTeslas(rawMag);

		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mag. X
	T_avg_pos /= MAGNETOMETER_CALIBRATION_SAMPLES;
	// De-activate +X
	setTorqueRodPwm(MAGNETORQUER_X,0);

	// Activate -X
	vTaskDelay(command_delay_ms);
	setTorqueRodPolarity(MAGNETORQUER_X, TR_POLARITY_NEG);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(MAGNETORQUER_X,255);
	vTaskDelay(activation_time_ms);
	// Sample Mag. X
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		uint16_t rawMag;
		getMagnetometerMeasurementsRaw(mag_id,buf);
		rawMag = 0;
		rawMag |=  ((uint16_t) buf[0]);
		rawMag |= (((uint16_t) buf[1]) << 8);

		T_avg_pos += convertMagDataRawToTeslas(rawMag);

		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mag. X
	T_avg_neg /= MAGNETOMETER_CALIBRATION_SAMPLES;
	// De-activate -X
	setTorqueRodPwm(MAGNETORQUER_X,0);

	// Calculate X offset
	mag_x_offset = (T_avg_pos + T_avg_neg) / 2;


	/*** Calibrate Mag. Y ***/
	// Activate +Y
	setTorqueRodPolarity(MAGNETORQUER_Y, TR_POLARITY_POS);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(MAGNETORQUER_Y, 255);
	vTaskDelay(activation_time_ms);
	// Sample Mag. Y
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		uint16_t rawMag;
		getMagnetometerMeasurementsRaw(mag_id,buf);
		rawMag = 0;
		rawMag |=  ((uint16_t) buf[2]);
		rawMag |= (((uint16_t) buf[3]) << 8);

		T_avg_pos -= convertMagDataRawToTeslas(rawMag); // NOTE MINUS-EQUALS (Y-axis is reversed on physical board)

		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mag. Y
	T_avg_pos /= MAGNETOMETER_CALIBRATION_SAMPLES;

	// Activate -Y
	setTorqueRodPwm(MAGNETORQUER_Y,0);
	vTaskDelay(command_delay_ms);
	setTorqueRodPolarity(MAGNETORQUER_Y, TR_POLARITY_NEG);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(MAGNETORQUER_Y,255);
	vTaskDelay(activation_time_ms);
	// Sample Mag. Y
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		uint16_t rawMag;
		getMagnetometerMeasurementsRaw(mag_id,buf);
		rawMag = 0;
		rawMag |=  ((uint16_t) buf[2]);
		rawMag |= (((uint16_t) buf[3]) << 8);

		T_avg_pos -= convertMagDataRawToTeslas(rawMag);  // NOTE MINUS-EQUALS (Y-axis is reversed on physical board)

		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mag. Y
	T_avg_neg /= MAGNETOMETER_CALIBRATION_SAMPLES;

	// Calculate Y offset
	mag_y_offset = (T_avg_pos + T_avg_neg) / 2;


	/*** Calibrate Mag. Z ***/
	// Activate +X
	setTorqueRodPolarity(MAGNETORQUER_Z, TR_POLARITY_POS);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(MAGNETORQUER_Z, 255);
	vTaskDelay(activation_time_ms);
	// Sample Mag. Z
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		uint16_t rawMag;
		getMagnetometerMeasurementsRaw(mag_id,buf);
		rawMag = 0;
		rawMag |=  ((uint16_t) buf[4]);
		rawMag |= (((uint16_t) buf[5]) << 8);

		T_avg_pos += convertMagDataRawToTeslas(rawMag);

		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mag. Z
	T_avg_pos /= MAGNETOMETER_CALIBRATION_SAMPLES;
	// De-activate +Z
	setTorqueRodPwm(MAGNETORQUER_Z,0);

	// Activate -Z
	vTaskDelay(command_delay_ms);
	setTorqueRodPolarity(MAGNETORQUER_Z, TR_POLARITY_NEG);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(MAGNETORQUER_Z,255);
	vTaskDelay(activation_time_ms);
	// Sample Mag. X
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		uint16_t rawMag;
		getMagnetometerMeasurementsRaw(mag_id,buf);
		rawMag = 0;
		rawMag |=  ((uint16_t) buf[4]);
		rawMag |= (((uint16_t) buf[5]) << 8);

		T_avg_pos += convertMagDataRawToTeslas(rawMag);

		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mag. Z
	T_avg_neg /= MAGNETOMETER_CALIBRATION_SAMPLES;
	// De-activate -Z
	setTorqueRodPwm(MAGNETORQUER_Z,0);

	// Calculate X offset
	mag_z_offset = (T_avg_pos + T_avg_neg) / 2;

	return status;
}

AdcsDriverError_t CalibrateMagnetometerSingleTorqueRod(MagnetometerId_t mag_id)
{
	const TickType_t sample_delay_ms = 10;
	const TickType_t command_delay_ms = 10;
	const TickType_t activation_time_ms = 200;
	const uint8_t calibration_pwm = 0;
	const MagnetorquerID_t calibration_rod = MAGNETORQUER_Y;

	uint8_t i,j;
	uint8_t buf[6];
	AdcsDriverError_t status = ADCS_DRIVER_NO_ERROR;

	float 	 T_avg_pos[3] = {0.0};
	float 	 T_avg_neg[3] = {0.0};

	/*** Turn off all torque rods ***/
	setTorqueRodPwm(MAGNETORQUER_X, 0);
	setTorqueRodPwm(MAGNETORQUER_Y, 0);
	setTorqueRodPwm(MAGNETORQUER_Z, 0);
	vTaskDelay(TORQUE_ROD_DECAY_TIME_MS);

	/*** Calibrate with torque rod X ***/
	// Activate +X
	setTorqueRodPolarity(calibration_rod, TR_POLARITY_POS);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(calibration_rod, calibration_pwm);
	vTaskDelay(activation_time_ms);
	// Sample Mags
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		getMagnetometerMeasurementsRaw(mag_id,buf);

		for(j=0; j < 3; j++)
		{
			uint16_t rawMag;
			rawMag = 0;
			rawMag |=  ((uint16_t) buf[2*j]);
			rawMag |= (((uint16_t) buf[2*j+1]) << 8);
			float magConv = convertMagDataRawToTeslas(rawMag);
			T_avg_pos[j] += mag_sign_flip[i] * magConv;
		}
		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mags
	for(i=0; i < 3; i++)
	{
		T_avg_pos[i] /= MAGNETOMETER_CALIBRATION_SAMPLES;
	}
	// De-activate +X
	setTorqueRodPwm(calibration_rod,0);

	// Activate -X
	vTaskDelay(command_delay_ms);
	setTorqueRodPolarity(calibration_rod, TR_POLARITY_NEG);
	vTaskDelay(command_delay_ms);
	setTorqueRodPwm(calibration_rod,calibration_pwm);
	vTaskDelay(activation_time_ms);
	// Sample Mags
	for(i=0; i < MAGNETOMETER_CALIBRATION_SAMPLES; i++)
	{
		uint16_t rawMag;
		getMagnetometerMeasurementsRaw(mag_id,buf);

		for(j=0; j < 3; j++)
		{
			uint16_t rawMag;
			rawMag = 0;
			rawMag |=  ((uint16_t) buf[2*j]);
			rawMag |= (((uint16_t) buf[2*j+1]) << 8);
			float magConv = convertMagDataRawToTeslas(rawMag);
			T_avg_neg[j] += mag_sign_flip[i] * magConv;
		}

		vTaskDelay(sample_delay_ms);
	}
	// Calculate average Mags
	for(i=0; i < 3; i++)
	{
		T_avg_neg[i] /= MAGNETOMETER_CALIBRATION_SAMPLES;
	}
	// De-activate -X
	setTorqueRodPwm(calibration_rod,0);

	// Calculate X offset
	for(i=0; i < 3; i++)
		mag_offsets[i] = (T_avg_pos[i] + T_avg_neg[i]) / 2;

	// Store offsets
	mag_x_offset = mag_offsets[0];
	mag_y_offset = mag_offsets[1];
	mag_z_offset = mag_offsets[2];

	return status;
}


void getEclipseBounds(Calendar_t * lowerBound, Calendar_t * upperBound)
{
	memcpy(lowerBound,&eclipseBounds[0],sizeof(Calendar_t));
	memcpy(upperBound,&eclipseBounds[1],sizeof(Calendar_t));
}

void setEclipseBounds(Calendar_t * lowerBound, Calendar_t * upperBound)
{

	memcpy(&eclipseBounds[0],lowerBound,sizeof(Calendar_t));
	memcpy(&eclipseBounds[1],upperBound,sizeof(Calendar_t));
}

const uint8_t num_days_per_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};

void calendarRollOver(Calendar_t * time)
{
	uint8_t rollOver = 0;
	// seconds
	rollOver 		= time->second / 60;
	time->second 	= time->second % 60;
	time->minute 	+= rollOver;
	// minutes
	rollOver 		= time->minute / 60;
	time->minute 	= time->minute % 60;
	time->hour 		+= rollOver;
	// hour
	rollOver 		= time->hour / 24;
	time->hour 		= time->hour % 24;
	time->day 		+= rollOver;
	// day
	uint8_t days_max = num_days_per_month[time->day-1];
	if(time->month == 2 && time->year == 24)
		days_max = 29;
	rollOver 		= time->day / (days_max + 1);
	time->day 		= time->day % (days_max + 1);
	if(time->day == 0)
		time->day = 1;
	time->month 	+= rollOver;
	// month
	rollOver 		= time->month / 13;
	time->month 	= time->month % 13;
	if(time->month == 0)
		time->month = 1;
	time->year 		+= rollOver;

}

bool spacecraftInEclipse(void)
{
	Calendar_t current_time = {0};
	MSS_RTC_get_calendar_count(&current_time); // get current time
	bool after_lower_bound  = compare_time(&current_time,&eclipseBounds[0]) == 1;
	bool before_upper_bound = compare_time(&eclipseBounds[1],&current_time) == 1;
	// Update if eclipse passed
	if(!before_upper_bound)
	{
		eclipseBounds[0].hour 	+= SPACECRAFT_ORBIT_TIME_HOURS;
		eclipseBounds[0].minute += SPACECRAFT_ORBIT_TIME_MINUTES;
		eclipseBounds[0].second += SPACECRAFT_ORBIT_TIME_SECONDS;
		calendarRollOver(&eclipseBounds[0]);

		eclipseBounds[1].hour 	+= SPACECRAFT_ORBIT_TIME_HOURS;
		eclipseBounds[1].minute += SPACECRAFT_ORBIT_TIME_MINUTES;
		eclipseBounds[1].second += SPACECRAFT_ORBIT_TIME_SECONDS;
		calendarRollOver(&eclipseBounds[1]);

	}
	return after_lower_bound && before_upper_bound;
}

bool sunSensorsOutOfRange(double x_comp, double z_comp)
{
	bool x_is_in_range = abs(x_comp) < sun_angle_max_vector_length;
	bool z_is_in_range = abs(z_comp) < sun_angle_max_vector_length;
	return x_is_in_range || z_is_in_range;
}











