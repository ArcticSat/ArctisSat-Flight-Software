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

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define ADCS_GYRO_DATA_SIZE  6
#define ADCS_MAGNETORQUER_DATA_SIZE  6
#define ADCS_SUN_SENSOR_DATA_SIZE   164
#define ADCS_TELEMETRY_TOTAL_SIZE   (ADCS_MAGNETORQUER_DATA_SIZE + ADCS_GYRO_DATA_SIZE + ADCS_SUN_SENSOR_DATA_SIZE)
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
AdcsDriverError_t getGyroMeasurements(GyroId_t gyroNumber, uint8_t * gyroMeasurements);
AdcsDriverError_t getMagnetometerMeasurements(MagnetometerId_t magnetometerNumber, uint8_t * magnetometerMeasurements);
AdcsDriverError_t getSunSensorMeasurements(uint8_t * measurements);



#endif //ADCS_DRIVER_H_
