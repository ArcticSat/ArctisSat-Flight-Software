/*
 * sun_pointing.c
 *
 *  Created on: Jan. 2, 2023
 *      Author: codyauch
 */

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "application/sun_pointing.h"
#include "main.h"
#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include "timers.h"
#include "drivers/device/adcs_driver.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Phase period settings
#define SAMPLE_SUN_SENSOR_DUR 1
#define SAMPLE_MAGS_CONTROL_DUR 4
#define CYCLE_DUR 10
// Number of samples by sensor type
#define NUM_SAMPLE_LOOPS_SUN_SENSOR			5
#define NUM_SAMPLE_LOOPS_GYROSCOPE			5
#define NUM_SAMPLE_LOOPS_MAGNETOMETER		5

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Sun Pointing State Enum
typedef enum  {
    SAMPLE_SUN_SENSOR_GYROS = SAMPLE_SUN_SENSOR_DUR,
    SAMPLE_MAGS_CONTROL = SAMPLE_MAGS_CONTROL_DUR,
    COMMAND_TORQ_RODS = CYCLE_DUR
}SUN_POINTING_STATES;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Sun Pointing Task Timer
TimerHandle_t sunPointingTimer;
// Sensor select
volatile enumSunSensor ss_x_select = SUN_SENSOR_1; // TODO: confirm value
volatile enumSunSensor ss_z_select = SUN_SENSOR_2; // TODO: confirm value
volatile GyroId_t gyro_select = GYRO_1;
volatile MagnetometerId_t mag_select = MAGNETOMETER_1;
// Sampling buffers
volatile uint8_t ss_x_buf[NUM_SAMPLE_LOOPS_MAGNETOMETER][ADCS_SUN_SENSOR_DATA_SIZE];
volatile uint8_t ss_z_buf[NUM_SAMPLE_LOOPS_MAGNETOMETER][ADCS_SUN_SENSOR_DATA_SIZE];
volatile uint8_t gyro_buf[NUM_SAMPLE_LOOPS_GYROSCOPE][ADCS_GYRO_RAW_DATA_SIZE_BYTES];
volatile uint8_t mag_buf[NUM_SAMPLE_LOOPS_MAGNETOMETER][ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES];
// Eclipse variable
volatile bool eclipse = false;
// Control gains
volatile const float rate_gain = 0.0004;
volatile float sun_gain = 0.0;
// Target rate
volatile const float target_rate = 0.0175; // radians/s
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void SunPointingP1( void );
void SunPointingP2( void );
void SunPointingP3( void );
void vHandleTimer(TimerHandle_t xTimer);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

void SunPointing( void )
{
    sunPointingTimer = xTimerCreate("sunPointingTimer", pdMS_TO_TICKS(100), pdTRUE, ( void * ) SAMPLE_SUN_SENSOR_GYROS, vHandleTimer);

    if(sunPointingTimer == NULL)
    {
        while(1);
    }
    else
    {
        xTimerStart(sunPointingTimer, portMAX_DELAY);
    }
    for(;;)
    {
        while(pvTimerGetTimerID(sunPointingTimer) == SAMPLE_SUN_SENSOR_GYROS)
        {
            SunPointingP1();
        }
        while(pvTimerGetTimerID(sunPointingTimer) == SAMPLE_MAGS_CONTROL)
        {
            SunPointingP2();
        }
        while(pvTimerGetTimerID(sunPointingTimer) == COMMAND_TORQ_RODS)
        {
            SunPointingP3();
        }
    }
}

void SunPointingP1( void )
{
    // Gyro Measurements & Conversion
    /*
        - Measurement (using adcs_driver.c function getGyroMeasurements) "returns" a 6-element array of uint8_ts (done)
        - Create another function, which will convert the above array, into 3 floats in units of mdps
            - Concatenate the high/low bytes for x,y,z (first into a uint16_t? or directly into a float) (a3g4250d_angular_rate_raw_get from ADCS code)
            - Convert to mdps (a3g4250d_from_fs245dps_to_mdps from ADCS code)
        - Result from this, should be a direct input into the control equation
    */

    // Sun Sensor Measurements & Conversion
    /*
        - Measurement (using adcs_driver.c function getSunSensorMeasurements) "returns" a 6-element array of uint8_ts (done)
            - Add function to pull out solely pixel data
        - What is the input to the control equation, and how to we convert from raw data??
    */


    // Main body
    /*
        - Turn the torque rods off
        - Loop SAMPLING_LOOPS (100 for now) times
        - In each loop:
            - Do 1 gyro measurement and conversion
            - Do 1 mag measurement and conversion
            - Do 1 sun sensor measurement and conversion
    */

    /*** Turn off torque rods ***/
    setTorqueRodState(TORQUE_ROD_1, TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_2, TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_3, TR_STATE_OFF);

    /*** TODO (sun-pointing): Poll the back panels to check if Y- is facing the sun ***/

    /*** Sampling ***/
    AdcsDriverError_t adcs_status;
    int i;
    // Sample sun sensors
    for(i=0; i<NUM_SAMPLE_LOOPS_SUN_SENSOR; i++)
    {
    	// TODO: adcs error handling
    	// Get Y-angle
    	adcs_status = sunSensorSelect(ss_x_select);
    	adcs_status = getSunSensorMeasurementsRaw(ss_x_buf[i]);
    	// Get Z-angle
		adcs_status = sunSensorSelect(ss_z_select);
		adcs_status = getSunSensorMeasurementsRaw(ss_z_buf[i]);
    }
    // Sample gyroscope
    for(i=0; i<NUM_SAMPLE_LOOPS_GYROSCOPE; i++)
    {
    	// TODO: adcs error handling
    	adcs_status = getMagnetometerMeasurementsRaw(gyro_select, gyro_buf[i]);
    }

}

void SunPointingP2( void )
{
    AdcsDriverError_t adcs_status;
    int i;
    float ss_x_sum,ss_z_sum;
    float ss_x, ss_y;
    float gyro_x_sum, gyro_y_sum, gyro_z_sum;
    float gyro_x, gyro_y, gyro_
    float mag_x_sum, mag_y_sum, mag_z_sum;
    float mag_x, mag_y, mag_z;
	/*** Sample magnetometers ***/
    for(i=0; i<NUM_SAMPLE_LOOPS_MAGNETOMETER; i++)
    {
    	// TODO: adcs error handling
    	adcs_status = getGyroMeasurementsRaw(gyro_select,gyro_buf[i]);
    }

	/*** Convert all sensor samples from raw to proper units ***/
	// TODO (sun-pointing): Convert sun sensor samples to unit vector
	// Convert gyroscope samples to radians per second

	// Convert magnetometer samples to gyroscopes

	/*** TODO (sun-pointing): Check if we are in eclipse ***/

	/*** Check to see if the sun sensors are lit (if not, then we are backwards) ***/
	if(eclipse)
	{
		sun_gain = 0.00001;
	}
	else
	{
		sun_gain = 0.0;
	}
	/*** Compute the rate error ***/
//	float rate_error_x =
}

void SunPointingP3( void ){}

void vHandleTimer(TimerHandle_t xTimer)
{
    int currentID = pvTimerGetTimerID(xTimer);
    currentID++;
    currentID %= CYCLE_DUR;

    if(currentID <= SAMPLE_SUN_SENSOR_DUR)
    {
        currentID = SAMPLE_SUN_SENSOR_GYROS;
    }
    else if(currentID <= SAMPLE_MAGS_CONTROL_DUR)
    {
        currentID = SAMPLE_MAGS_CONTROL;
    }
    else
    {
        currentID = COMMAND_TORQ_RODS;
    }

    vTimerSetTimerID(xTimer, currentID);
}
