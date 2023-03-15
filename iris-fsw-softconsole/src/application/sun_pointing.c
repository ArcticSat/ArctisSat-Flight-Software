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
#include "drivers/subsystems/eps_driver.h"
#include "application/cdh.h"

#include <math.h>

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
// Back panel polling
#define BACKPANEL_SA_POLLING_LOOPS				10

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
// Back panel solar array poll loop counter
uint8_t backpanel_loop_counter = 0;
// Sensor select
volatile enumSunSensor ss_x_select = SUN_SENSOR_1; // TODO: confirm value
volatile enumSunSensor ss_z_select = SUN_SENSOR_2; // TODO: confirm value
volatile GyroId_t gyro_select = GYRO_1;
volatile MagnetometerId_t mag_select = MAGNETOMETER_1;
// Sampling buffers
volatile uint8_t ss_x_buf[NUM_SAMPLE_LOOPS_SUN_SENSOR][ADCS_SUN_SENSOR_DATA_SIZE];
volatile uint8_t ss_z_buf[NUM_SAMPLE_LOOPS_SUN_SENSOR][ADCS_SUN_SENSOR_DATA_SIZE];
volatile uint8_t gyro_buf[NUM_SAMPLE_LOOPS_GYROSCOPE][ADCS_GYRO_RAW_DATA_SIZE_BYTES];
volatile uint8_t mag_buf[NUM_SAMPLE_LOOPS_MAGNETOMETER][ADCS_MAGNETOMETER_RAW_DATA_SIZE_BYTES];
// Sample validity
volatile bool ss_sample_valid[NUM_SAMPLE_LOOPS_SUN_SENSOR] = {false};
volatile bool gyro_sample_valid[NUM_SAMPLE_LOOPS_GYROSCOPE] = {false};
volatile bool mag_sample_valid[NUM_SAMPLE_LOOPS_MAGNETOMETER] = {false};
// Number of valid samples
volatile uint8_t num_valid_ss_samples = 0;
volatile uint8_t num_valid_gyro_samples = 0;
volatile uint8_t num_valid_mag_samples = 0;
// Eclipse variable
volatile bool eclipse = false;
// Control gains
volatile const float rate_gain = 0.0004;
volatile float sun_gain = 0.0;
// Target rate
volatile const float target_rate_y = 0.0175; // radians/s
// Dipole command
float dipole_cmd_x, dipole_cmd_y, dipole_cmd_z;
// Saturation limit
volatile const float magt_sat = 0.1731; // Amp-m^2
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

    int i;

    /*** Turn off torque rods ***/
    setTorqueRodState(TORQUE_ROD_1, TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_2, TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_3, TR_STATE_OFF);

    /*** Poll the back panels to check if Y- is facing the sun ***/
    if(backpanel_loop_counter == BACKPANEL_SA_POLLING_LOOPS)
    {
    	pollBackSolarPanels();
    	backpanel_loop_counter = 0;
    }
    else
    {
    	backpanel_loop_counter++;
    }

    /*** Clear validity variables ***/
    for(i=0; i < NUM_SAMPLE_LOOPS_SUN_SENSOR; i++) ss_sample_valid[i] = false;
    for(i=0; i < NUM_SAMPLE_LOOPS_GYROSCOPE; i++) gyro_sample_valid[i] = false;
    for(i=0; i < NUM_SAMPLE_LOOPS_MAGNETOMETER; i++) mag_sample_valid[i] = false;
    num_valid_ss_samples = 0;
    num_valid_gyro_samples = 0;
    num_valid_mag_samples = 0;

    /*** Sampling ***/
    // Sample sun sensors
    for(i=0; i<NUM_SAMPLE_LOOPS_SUN_SENSOR; i++)
    {
    	if(ADCS_DRIVER_NO_ERROR != sunSensorSelect(ss_x_select)) continue;
    	if(ADCS_DRIVER_NO_ERROR != getSunSensorMeasurementsRaw(ss_x_buf[i])) continue;
    	// Get Z-angle
    	if(ADCS_DRIVER_NO_ERROR != sunSensorSelect(ss_z_select)) continue;
    	if(ADCS_DRIVER_NO_ERROR != getSunSensorMeasurementsRaw(ss_z_buf[i])) continue;
    	// Update valid samples
    	ss_sample_valid[i] = true;
    	num_valid_ss_samples++;
    }
    // Sample gyroscope
    for(i=0; i<NUM_SAMPLE_LOOPS_GYROSCOPE; i++)
    {
    	if(ADCS_DRIVER_NO_ERROR != getMagnetometerMeasurementsRaw(gyro_select, gyro_buf[i])) continue;
    	// Update number of valid samples
    	gyro_sample_valid[i] = true;
    	num_valid_gyro_samples++;
    }

}

void SunPointingP2( void )
{
    AdcsDriverError_t adcs_status;
    int i;
    float ss_x_sum,ss_z_sum;
    float ss_x, ss_z;
    float gyro_x_sum, gyro_y_sum, gyro_z_sum;
    float gyro_x, gyro_y, gyro_z;
    float mag_x_sum, mag_y_sum, mag_z_sum;
    float mag_x, mag_y, mag_z;
    float rate_error_x, rate_error_y, rate_error_z;
    float sun_error_x, sun_error_z;
    float torque_command_x, torque_command_y, torque_command_z;
    float B_scale;

	/*** Sample magnetometers ***/
    for(i=0; i<NUM_SAMPLE_LOOPS_MAGNETOMETER; i++)
    {
    	if(ADCS_DRIVER_NO_ERROR != getGyroMeasurementsRaw(gyro_select,gyro_buf[i])) continue;
    	// Update number of valid samples
    	mag_sample_valid[i] = true;
    	num_valid_mag_samples++;
    }

	/*** Convert all sensor samples from raw to proper units ***/
	// TODO (sun-pointing): Convert sun sensor samples to unit vector
    ss_x_sum = 0.0;
    ss_z_sum = 0.0;
    for(i=0; i<NUM_SAMPLE_LOOPS_SUN_SENSOR; i++)
    {
    	// TODO: adcs error handling - validity check
    	// Convert X-angle
    	// Convert Z-angle
    }

	// Convert gyroscope samples to radians per second
    gyro_x_sum = 0.0;
    gyro_y_sum = 0.0;
    gyro_z_sum = 0.0;
    for(i=0; i<NUM_SAMPLE_LOOPS_GYROSCOPE; i++)
    {
    	if(gyro_sample_valid[i])
    	{
			uint16_t rawGyro;
			// Gyro X conversion and sum
			rawGyro = 0;
			rawGyro |=  ((uint16_t) gyro_buf[0]);
			rawGyro |= (((uint16_t) gyro_buf[1]) << 8);
			gyro_x_sum += convertGyroDataRawToRadiansPerSecond(rawGyro);
			// Gyro Y conversion and sum
			rawGyro = 0;
			rawGyro |=  ((uint16_t) gyro_buf[2]);
			rawGyro |= (((uint16_t) gyro_buf[3]) << 8);
			gyro_y_sum += convertGyroDataRawToRadiansPerSecond(rawGyro);
			// Gyro Z conversion and sum
			rawGyro = 0;
			rawGyro |=  ((uint16_t) gyro_buf[4]);
			rawGyro |= (((uint16_t) gyro_buf[5]) << 8);
			gyro_z_sum += convertGyroDataRawToRadiansPerSecond(rawGyro);
    	}
    }
	// Convert magnetometer samples to Tesla
    mag_x_sum = 0.0;
    mag_y_sum = 0.0;
    mag_z_sum = 0.0;
    for(i=0; i<NUM_SAMPLE_LOOPS_MAGNETOMETER; i++)
    {
    	if(mag_sample_valid[i])
    	{
			uint16_t rawMag;
			// Mag X conversion and sum
			rawMag = 0;
			rawMag |=  ((uint16_t) mag_buf[0]);
			rawMag |= (((uint16_t) mag_buf[1]) << 8);
			mag_x_sum += convertMagDataRawToTeslas(rawMag);
			// Mag Y conversion and sum
			rawMag = 0;
			rawMag |=  ((uint16_t) mag_buf[2]);
			rawMag |= (((uint16_t) mag_buf[3]) << 8);
			mag_y_sum += convertMagDataRawToTeslas(rawMag);
			// Mag Z conversion and sum
			rawMag = 0;
			rawMag |=  ((uint16_t) mag_buf[4]);
			rawMag |= (((uint16_t) mag_buf[5]) << 8);
			mag_z_sum += convertMagDataRawToTeslas(rawMag);
    	}
    }

    /*** Calculate average converted values ***/
    // TODO: error handling - what if no valid samples??? Reset ADCS and return to P1?
    // Sun-sensor averages
    if(num_valid_ss_samples > 0)
    {
		ss_x = ss_x_sum / num_valid_ss_samples;
		ss_z = ss_z_sum / num_valid_ss_samples;
    }
    // Gyro average
    if(num_valid_gyro_samples > 0)
    {
		gyro_x = gyro_x_sum / num_valid_gyro_samples;
		gyro_y = gyro_y_sum / num_valid_gyro_samples;
		gyro_z = gyro_z_sum / num_valid_gyro_samples;
    }
    // Mag average
    if(num_valid_mag_samples > 0)
    {
		mag_x = mag_x_sum / num_valid_mag_samples;
		mag_y = mag_y_sum / num_valid_mag_samples;
		mag_z = mag_z_sum / num_valid_mag_samples;
    }

	/*** Check to see if we are backwards (not needed) ***/

    /*** Define the Sun gain ***/
    // Note: we only need one valid ss sample
    if(num_valid_ss_samples > 0)
    {
    	sun_gain = 0.00001;
    }
    else
    {
    	sun_gain = 0.0;
    }

	/*** Compute the rate error (rad/s) ***/
    rate_error_x = -gyro_x;
    rate_error_y = target_rate_y - gyro_y;
    rate_error_z = -gyro_z;

    /*** Compute the sun error (unitless) ***/
    sun_error_x = ss_z;
    // Did not calculate sun_error_y because we don't need it
    sun_error_z = -ss_x;

    /*** Compute the torque command (Newton metres) ***/
    torque_command_x = (rate_error_x * rate_gain) + (sun_error_x * sun_gain);
    torque_command_y = (rate_error_y * rate_gain);
    torque_command_z = (rate_error_z * rate_gain) + (sun_error_z * sun_gain);

    /*** Compute the dipole command ***/
    B_scale = 1/(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z); // 1/Tesla^2
    dipole_cmd_x = B_scale*(mag_y*torque_command_z - mag_z*torque_command_y); // Amp-m^2
    dipole_cmd_y = B_scale*(mag_z*torque_command_x - mag_x*torque_command_z); // Amp-m^2
    dipole_cmd_z = B_scale*(mag_x*torque_command_y - mag_y*torque_command_x); // Amp-m^2
}

void SunPointingP3( void )
{
	float max_cmd;
	float magt_scale;
	float scaled_dipole_cmd_x, scaled_dipole_cmd_y, scaled_dipole_cmd_z;
	/*** Find the max abs dipole command ***/
	max_cmd = abs(dipole_cmd_x);
	if(abs(dipole_cmd_y) > max_cmd)
	{
		max_cmd = abs(dipole_cmd_y);
	}
	if(abs(dipole_cmd_z) > max_cmd)
	{
		max_cmd = abs(dipole_cmd_z);
	}

	/*** Compute the scaling factor ***/
	if(max_cmd > magt_sat)
	{
		magt_scale = magt_sat / max_cmd;
	}
	else
	{
		magt_scale = 1.0;
	}

	/*** Scale the commands ***/
	scaled_dipole_cmd_x = dipole_cmd_x*magt_scale;
	scaled_dipole_cmd_y = dipole_cmd_y*magt_scale;
	scaled_dipole_cmd_z = dipole_cmd_z*magt_scale;

	/*** TODO: Send the scaled commands to magnetorquers if we are not backwards ***/
}

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