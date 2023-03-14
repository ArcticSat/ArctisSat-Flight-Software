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
#include <math.h> //If someone can implement an arctan without math.h that would be sweet.

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// SPI parameters
#define ADCS_GYRO_RAW_DATA_SIZE_BYTES  6
#define ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES  6
#define ADCS_SUN_SENSOR_DATA_SIZE   164
#define ADCS_TELEMETRY_TOTAL_SIZE   (ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES + ADCS_GYRO_RAW_DATA_SIZE_BYTES + ADCS_SUN_SENSOR_DATA_SIZE)
// Raw gyro data conversion
#define A3G4250D_FULL_SCALE_MAX 8.0
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
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMERATIONS AND ENUMERATION TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum {
	ADCS_DRIVER_NO_ERROR,
	ADCS_ERROR_BAD_ACK,
	ADCS_ERROR_BAD_ID
} AdcsDriverError_t;

typedef enum{
    MAGNETORQUER_X,
    MAGNETORQUER_Y,
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
	SUN_SENSOR_1=1,
	SUN_SENSOR_2,		//2
	SUN_SENSOR_3,		//3
	SUN_SENSOR_4,		//4
} enumSunSensor;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

// Utilities
AdcsDriverError_t adcs_init_driver(void);
AdcsDriverError_t adcsTxRx(uint8_t * tx_data, uint16_t tx_size, uint8_t * rx_data, uint16_t rx_size);
// Utility commands
AdcsDriverError_t pingAdcs(void);
AdcsDriverError_t adcsSyncSpiCommand(uint8_t cmd_id);
AdcsDriverError_t adcsSyncSpi(void);
// Torque Rod commands
AdcsDriverError_t setTorqueRodState(TorqueRodId_t rod_number, TortqueRodState_t rod_state);
AdcsDriverError_t setTorqueRodPolarity(TorqueRodId_t cmd_id, TortqueRodPolarity_t polarity);
AdcsDriverError_t setTorqueRodPwm(TorqueRodId_t cmd_id, uint8_t pwm);
// Sensor Polling commands
AdcsDriverError_t setGyroI2cAddress(uint8_t addr);
AdcsDriverError_t getGyroMeasurementsGenericRaw(uint8_t * gyroMeasurements);
AdcsDriverError_t getGyroMeasurementsRaw(GyroId_t gyroNumber, uint8_t * gyroMeasurements);
AdcsDriverError_t getMagnetometerMeasurementsRaw(MagnetometerId_t magnetometerNumber, uint8_t * magnetometerMeasurements);
AdcsDriverError_t sunSensorSelect(enumSunSensor sunSensor);
AdcsDriverError_t getSunSensorMeasurementsRaw(uint8_t * measurements);
// Raw sensor data conversion
float a3g4250d_from_fs245dps_to_mdps(int16_t lsb);
float mmc5883ma_from_fs8G_to_mG(uint16_t mag_fs_raw);
uint16_t AngleDecompose(uint8_t *RXBuff,uint8_t selec);
// Application-level sensor polling
AdcsDriverError_t getGyroscopeDataRadians(GyroId_t gyroNumber, float * gyroData);
AdcsDriverError_t getMagnetometerDataTeslas(MagnetometerId_t magnetometerNumber, float * magnetometerData);
AdcsDriverError_t getSunAngle(uint8_t * measurements);



#endif //ADCS_DRIVER_H_
