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

#ifndef ADCS_DRIVER_H_
#define ADCS_DRIVER_H_

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "drivers/protocol/spi.h"
#include "main.h"
#include "drivers/device/rtc/rtc_common.h"
#include <math.h> //If someone can implement an arctan without math.h that would be sweet.

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Debug settings
//#define SIM_VALUES_1
//#define SIM_VALUES_2

#ifdef SIM_VALUES_1
#define BACKWARDS_SIM_ENG_VALUE			0
#define GYRO_SIM_ENG_VALUE 				0.0					// rad/s
#define GYRO_X_SIM_ENG_VALUE 			0.01790779			// rad/s
#define GYRO_Y_SIM_ENG_VALUE 			0.01698973			// rad/s
#define GYRO_Z_SIM_ENG_VALUE 			0.01677946			// rad/s
#define MAG_SIM_ENG_VALUE				0.0					// Teslas
#define MAG_X_SIM_ENG_VALUE				-0.00040812			// Teslas
#define MAG_Y_SIM_ENG_VALUE				0.000197040			// Teslas
#define MAG_Z_SIM_ENG_VALUE				-4.9727735e-5		// Teslas
#define SUN_ANGLE_SIM_ENG_VALUE 		 0					// unitless
#define SUN_VECT_X_SIM_ENG_VALUE 		-0.16555641			// unitless
#define SUN_VECT_Z_SIM_ENG_VALUE 		0.35368141			// unitless
#define TORQUE_ROD_SIM
#elif defined(SIM_VALUES_2)
#define BACKWARDS_SIM_ENG_VALUE			 0
#define GYRO_SIM_ENG_VALUE 				 0.0				// rad/s
#define GYRO_X_SIM_ENG_VALUE 			 1000				// rad/s
#define GYRO_Y_SIM_ENG_VALUE 			 1200				// rad/s
#define GYRO_Z_SIM_ENG_VALUE 			-2000 				// rad/s
#define MAG_SIM_ENG_VALUE				 0.0				// Teslas
#define MAG_X_SIM_ENG_VALUE				-0.0004114751		// Teslas
#define MAG_Y_SIM_ENG_VALUE				 0.00019760653		// Teslas
#define MAG_Z_SIM_ENG_VALUE				-9.963983400e-5		// Teslas
#define SUN_ANGLE_SIM_ENG_VALUE 		 0					// unitless
#define SUN_VECT_X_SIM_ENG_VALUE 		-0.1319210932		// unitless
#define SUN_VECT_Z_SIM_ENG_VALUE 		 0.29519842314		// unitless
#define TORQUE_ROD_SIM
#endif
// SPI parameters
#define ADCS_GYRO_RAW_DATA_SIZE_BYTES  6
#define ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES  6
#define ADCS_SUN_SENSOR_DATA_SIZE   164
#define ADCS_TELEMETRY_TOTAL_SIZE   (ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES + ADCS_GYRO_RAW_DATA_SIZE_BYTES + ADCS_SUN_SENSOR_DATA_SIZE)
// Axes
#define NUM_MAG_AXES 3
// Magnetometer calibration parameters
#define MAGNETOMETER_CALIBRATION_CYCLES_SECONDS		2*60
// Torque rod parameters
#define TORQUE_ROD_DECAY_TIME_MS 200
#define DIPOLE_SLOPE 0.04
#define MAX_DIPOLE 0.2
#define MAX_VOLTAGE 5.0
#define MAX_PWM 255.0
#define MIN_DETUMBLE_ROTATION_RPS 0.5
// Gyroscope data conversion
#define A3G4250D_LSB_DPS 0.00875
#define DPS_TO_RPS 0.017448
// Magnetometer data conversion
#define MAG_LSB 0.00025 //From datasheet - LSB is 0.25 miligauss
#define GAUSS_TO_TESLA_CONVERSION 1/10000 //1 tesla = 10000 gauss
// Raw sun sensor data conversion
#define NOP 0x00    //Idle command
#define CR 0xF0     //Chip reset
#define RT 0x68     //Read threshold
#define WT 0xCC     //Write threshold
#define SI 0xB8     //Start integration
#define SIL 0xB4    // Start integration long
#define RO1 0x9C    //ReadOut 1-bit
#define RO2 0x96    //ReadOut 1.5(2)-bit
#define RO4 0x93    //ReadOut 4-bit
#define RO8 0x99    //ReadOut 8-bit
#define TZ1 0xE8    //Test zebra pattern 1
#define TZ2 0xE4    //Test zebra pattern 2
#define TZ12 0xE2   //Test zebra pattern 1 & 2
#define TZ0 0xE1    //Test zebra pattern 0
#define SM 0xC6     //Sleep mode
#define WU 0xC3     //Wake-up
#define MASK_HEIGHT 2.06    // h in the paper (mm)
#define REF_PIXEL 62        // reference pixel
#define PIXEL_LENGTH 50e-3  // pixel length (mm)
#define sunBufferOffset 13	// Index of first pixel in raw sun data
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMERATIONS AND ENUMERATION TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum {
	ADCS_DRIVER_NO_ERROR,
	ADCS_ERROR_BAD_ACK,
	ADCS_ERROR_BAD_ID,
	ADCS_ERROR_BAD_DATA
} AdcsDriverError_t;

typedef enum{
    MAGNETORQUER_Y,
    MAGNETORQUER_X,
    MAGNETORQUER_Z
}MagnetorquerID_t;

// Torque Rod state
typedef enum
{
	TR_STATE_OFF,
	TR_STATE_ON
} TortqueRodState_t;
// Torque Rod polarity
typedef enum
{
	TR_POLARITY_NEG,
	TR_POLARITY_POS
} TortqueRodPolarity_t;
// Torque Rod IDs
typedef enum
{
	TORQUE_ROD_1 = 1,
	TORQUE_ROD_2,
	TORQUE_ROD_3
} TorqueRodId_t;
// Gyro IDs
typedef enum
{
	GYRO_1 = 1,
	GYRO_2
} GyroId_t;
// Magnetometer IDs
typedef enum
{
	MAGNETOMETER_1 = 1,
	MAGNETOMETER_2
} MagnetometerId_t;

typedef enum
{
	SUN_SENSOR_PRIMARY_Z=1,
	SUN_SENSOR_PRIMARY_X,		//2
	SUN_SENSOR_3,				//3
	SUN_SENSOR_4,				//4
} enumSunSensor;

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

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBAL VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Magnetometer calibration variables
extern float mag_offsets[3];
extern float mag_x_offset;
extern float mag_y_offset;
extern float mag_z_offset;
// Sun vector conversion
extern volatile float ss_x_sign_flip;
extern volatile float ss_z_sign_flip;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Testing
void vTestAdcsDriverInterface(void * pvParameters);
// Utilities
AdcsDriverError_t adcs_init_driver(void);
AdcsDriverError_t adcsTxRx(uint8_t * tx_data, uint16_t tx_size, uint8_t * rx_data, uint16_t rx_size);
// Utility commands
AdcsDriverError_t pingAdcs(void);
AdcsDriverError_t adcsSyncSpiCommand(uint8_t cmd_id);
AdcsDriverError_t adcsSyncSpi(void);
// Torque Rod commands
AdcsDriverError_t setTorqueRodState(MagnetorquerID_t rod_number, TortqueRodState_t rod_state);
AdcsDriverError_t setTorqueRodPolarity(MagnetorquerID_t rod_number, uint8_t polarity);
AdcsDriverError_t setTorqueRodPwm(MagnetorquerID_t cmd_id, uint8_t pwm);
/*** TODO: validity of sampled data (e.g. all zeros may indicate an SPI transfer error) ***/
// Sensor Polling commands
AdcsDriverError_t setGyroI2cAddress(uint8_t addr);
AdcsDriverError_t getGyroMeasurementsGenericRaw(uint8_t * gyroMeasurements);
AdcsDriverError_t getGyroMeasurementsRaw(GyroId_t gyroNumber, uint8_t * gyroMeasurements);
AdcsDriverError_t getMagnetometerMeasurementsRaw(MagnetometerId_t magnetometerNumber, uint8_t * magnetometerMeasurements);
AdcsDriverError_t sunSensorSelect(enumSunSensor sunSensor);
AdcsDriverError_t getSunSensorMeasurementsRaw(volatile uint8_t * measurements);
// Torque rod data conversion
double dipoleToVoltage(double dipole);
void MapTorqueRodCommand(double dipole, uint8_t * polarity, uint8_t * pwm);
// Raw sensor data conversion
float convertGyroDataRawToRadiansPerSecond(uint16_t rawGyro);
float convertMagDataRawToTeslas(uint16_t rawMag);
float AngleDecompose(uint8_t *RXBuff,uint8_t selec );
// Application-level sensor polling
AdcsDriverError_t getGyroscopeDataRadiansPerSecond(GyroId_t gyroNumber, float * gyroDataRps);
AdcsDriverError_t getMagnetometerDataTeslas(MagnetometerId_t magnetometerNumber, float * magnetometerData);
AdcsDriverError_t getSunAngle(uint8_t * measurements);
// Sensor calibration
AdcsDriverError_t CalibrateMagnetometer(MagnetometerId_t mag_id);
AdcsDriverError_t CalibrateMagnetometerSingleTorqueRod(MagnetometerId_t mag_id);
// Eclipse checking
void getEclipseBounds(Calendar_t * lowerBound, Calendar_t * upperBound);
void setEclipseBounds(Calendar_t * lowerBound, Calendar_t * upperBound);
bool spacecraftInEclipse(void);
bool spacecraftIsNotBackwards(double x_comp, double z_comp);



#endif //ADCS_DRIVER_H_
