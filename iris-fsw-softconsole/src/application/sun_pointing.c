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
#include "task.h"
#include "drivers/device/adcs_driver.h"
#include "drivers/subsystems/eps_driver.h"
#include "application/cdh.h"
#ifdef SUN_POINTING_DEBUG_TELEMETRY
#include "tasks/telemetry.h"
#endif

#include <math.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Phase period settings
#define SAMPLE_SUN_SENSOR_DUR 2
#define SAMPLE_MAGS_CONTROL_DUR 5
#define CYCLE_DUR 10
// Number of samples by sensor type
#define NUM_SAMPLE_LOOPS_SUN_SENSOR			5
#define NUM_SAMPLE_LOOPS_GYROSCOPE			5
#define NUM_SAMPLE_LOOPS_MAGNETOMETER		5
// Back panel polling
#define BACKPANEL_SA_POLLING_LOOPS				10
// Telemetry rates
#define SUN_POINTING_TM_RATE 5
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
} eSunPointingStates;
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
// Spacecraft is backwards variable
volatile bool backwards = false;
// Control gains
volatile const float rate_gain = 0.0004;
volatile float sun_gain = 0.0;
// Target rate
volatile const float target_rate_y = 0.017453292519943; // radians/s
// Dipole command
double torque_command_x, torque_command_y, torque_command_z;
double dipole_cmd_x, dipole_cmd_y, dipole_cmd_z;
volatile uint8_t polarity_x, polarity_y, polarity_z;
volatile uint8_t pwm_x, pwm_y, pwm_z;
// Saturation limit
volatile const float magt_sat = 0.1731; // Amp-m^2
// Debugging telemetry
#ifdef SUN_POINTING_DEBUG_TELEMETRY
telemetryPacket_t sun_pointing_pkt = {0};
uint8_t sun_pointing_pkt_count = 0;
#endif
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

eSunPointingStates determineSunPointingState(int inputID) {

	eSunPointingStates output;

    if(inputID < SAMPLE_SUN_SENSOR_GYROS) { //up to 200ms
        output = SAMPLE_SUN_SENSOR_GYROS;
    } else if(inputID < SAMPLE_MAGS_CONTROL) { //200 - 500 = 300ms
        output = SAMPLE_MAGS_CONTROL;
    } else { //1000 - 500 = 500ms
        output = COMMAND_TORQ_RODS;
    }

//    vTaskDelay(10);

    return output;
}

void vSunPointing( void * pvParameters )
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
        while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == SAMPLE_SUN_SENSOR_GYROS)
        {
            SunPointingP1();
        }
        while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == SAMPLE_MAGS_CONTROL)
        {
            SunPointingP2();
        }
        while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == COMMAND_TORQ_RODS)
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
	setTorqueRodPwm(TORQUE_ROD_1,0);
	setTorqueRodPwm(TORQUE_ROD_2,0);
	setTorqueRodPwm(TORQUE_ROD_3,0);

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

    /*** Wait ***/
    while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == SAMPLE_SUN_SENSOR_GYROS);
}

void SunPointingP2( void )
{
    AdcsDriverError_t adcs_status;
	int i;
	double ss_x_sum,ss_z_sum;
	double ss_x, ss_z;
	double gyro_x_sum, gyro_y_sum, gyro_z_sum;
	double gyro_x, gyro_y, gyro_z;
	double mag_x_sum, mag_y_sum, mag_z_sum;
	double mag_x, mag_y, mag_z;
	double rate_error_x, rate_error_y, rate_error_z;
	double sun_error_x, sun_error_z;
	double torque_command_x, torque_command_y, torque_command_z;
	double B_scale;
	double max_cmd;
	double magt_scale;
	double scaled_dipole_cmd_x, scaled_dipole_cmd_y, scaled_dipole_cmd_z;
	double voltage_x, voltage_y, voltage_z;

	/*** Sample magnetometers ***/
	for(i=0; i<NUM_SAMPLE_LOOPS_MAGNETOMETER; i++)
	{
		if(ADCS_DRIVER_NO_ERROR != getGyroMeasurementsRaw(gyro_select,gyro_buf[i])) continue;
		// Update number of valid samples
		mag_sample_valid[i] = true;
		num_valid_mag_samples++;
	}

	/*** Convert all sensor samples from raw to proper units ***/
	ss_x_sum = 0.0;
	ss_z_sum = 0.0;
	for(i=0; i<NUM_SAMPLE_LOOPS_SUN_SENSOR; i++)
	{
		if(ss_sample_valid[i])
		{
			// Convert X-angle
			ss_x_sum += sin(AngleDecompose(ss_x_buf[i],3));
			// Convert Z-angle
			ss_z_sum += sin(AngleDecompose(ss_z_buf[i],3));
		}
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
#ifndef SUN_VECT_X_SIM_ENG_VALUE
		ss_x = ss_x_sum / num_valid_ss_samples;
#else
		ss_x = SUN_VECT_X_SIM_ENG_VALUE;
#endif
#ifndef SUN_VECT_Z_SIM_ENG_VALUE
		ss_z = ss_z_sum / num_valid_ss_samples;
#else
		ss_z = SUN_VECT_Z_SIM_ENG_VALUE;
#endif
	}
	// Gyro average
	if(num_valid_gyro_samples > 0)
	{
#ifndef GYRO_X_SIM_ENG_VALUE
		gyro_x = gyro_x_sum / num_valid_gyro_samples;
#else
		gyro_x = GYRO_X_SIM_ENG_VALUE;
#endif
#ifndef GYRO_Y_SIM_ENG_VALUE
		gyro_y = gyro_y_sum / num_valid_gyro_samples;
#else
		gyro_y = GYRO_Y_SIM_ENG_VALUE;
#endif
#ifndef GYRO_Z_SIM_ENG_VALUE
		gyro_z = gyro_z_sum / num_valid_gyro_samples;
#else
		gyro_z = GYRO_Z_SIM_ENG_VALUE;
#endif
	}
	// Mag average
	if(num_valid_mag_samples > 0)
	{
#ifndef MAG_X_SIM_ENG_VALUE
		mag_x = mag_x_sum / num_valid_mag_samples;
#else
		mag_x = MAG_X_SIM_ENG_VALUE;
#endif
#ifndef MAG_Y_SIM_ENG_VALUE
		mag_y = mag_y_sum / num_valid_mag_samples;
#else
		mag_y = MAG_Y_SIM_ENG_VALUE;
#endif
#ifndef MAG_Z_SIM_ENG_VALUE
		mag_z = mag_z_sum / num_valid_mag_samples;
#else
		mag_z = MAG_Z_SIM_ENG_VALUE;
#endif
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

	/*** Calculate the scaled commands to magnetorquers (based on backwards) ***/
	backwards = spacecraftIsBackwards();
#ifdef BACKWARDS_SIM_ENG_VALUE
	backwards = BACKWARDS_SIM_ENG_VALUE;
#endif
	if(backwards)
	{
		MapTorqueRodCommand(magt_sat/2, &polarity_x, &pwm_x);
		MapTorqueRodCommand(magt_sat/2, &polarity_y, &pwm_y);
		MapTorqueRodCommand((magt_sat/2)*mag_z/mag_y, &polarity_z, &pwm_z);
	}
	else
	{
		MapTorqueRodCommand(scaled_dipole_cmd_x, &polarity_x, &pwm_x);
		MapTorqueRodCommand(scaled_dipole_cmd_y, &polarity_y, &pwm_y);
		MapTorqueRodCommand(scaled_dipole_cmd_z, &polarity_z, &pwm_z);
	}

    /*** Wait ***/
    while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == SAMPLE_MAGS_CONTROL);
}

void SunPointingP3( void )
{

#ifdef SUN_POINTING_DEBUG_TELEMETRY
	uint8_t data[58] = {0};
	sun_pointing_pkt.telem_id = CDH_SUN_POINTING_ID;
	sun_pointing_pkt.length = 58;
	sun_pointing_pkt.data = data;
	// Backwards
	memcpy(&data[0],&backwards,sizeof(uint8_t));
	// Number of valid samples
	memcpy(&data[1],&num_valid_ss_samples,sizeof(uint8_t));
	memcpy(&data[2],&num_valid_gyro_samples,sizeof(uint8_t));
	memcpy(&data[3],&num_valid_mag_samples,sizeof(uint8_t));
	// Torque command
	memcpy(&data[4],&torque_command_x,sizeof(torque_command_x));
	memcpy(&data[12],&torque_command_y,sizeof(torque_command_y));
	memcpy(&data[20],&torque_command_z,sizeof(torque_command_z));
	// Dipole command
	memcpy(&data[28],&dipole_cmd_x,sizeof(dipole_cmd_x));
	memcpy(&data[36],&dipole_cmd_y,sizeof(dipole_cmd_y));
	memcpy(&data[44],&dipole_cmd_z,sizeof(dipole_cmd_z));
	// Actuation polarity
	memcpy(&data[52],&polarity_x,sizeof(polarity_x));
	memcpy(&data[53],&polarity_y,sizeof(polarity_y));
	memcpy(&data[54],&polarity_z,sizeof(polarity_z));
	// PWM
	memcpy(&data[55],&pwm_x,sizeof(pwm_x));
	memcpy(&data[56],&pwm_y,sizeof(pwm_y));
	memcpy(&data[57],&pwm_z,sizeof(pwm_z));
	// Send telemetry
	if((SUN_POINTING_TM_RATE - 1) == sun_pointing_pkt_count)
	{
		int float_size = sizeof(float);
		int double_size = sizeof(double);
		uint8_t scale = (uint8_t) ((2.0 / 5.0) * 255.0);
		sendTelemetryAddr(&sun_pointing_pkt, GROUND_CSP_ADDRESS);
		sun_pointing_pkt_count = 0;
	}
	else
	{
		sun_pointing_pkt_count++;
	}
#endif

	/*** Send the scaled commands to magnetorquers ***/
	// Set polarity
	setTorqueRodPolarity(TORQUE_ROD_1,polarity_x);
	setTorqueRodPolarity(TORQUE_ROD_2,polarity_y);
	setTorqueRodPolarity(TORQUE_ROD_3,polarity_z);
	// Set PWM
	setTorqueRodPwm(TORQUE_ROD_1,pwm_x);
	setTorqueRodPwm(TORQUE_ROD_2,pwm_y);
	setTorqueRodPwm(TORQUE_ROD_3,pwm_z);

    /*** Wait ***/
    while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == COMMAND_TORQ_RODS);
}

void vHandleTimer(TimerHandle_t xTimer)
{
    int currentID = pvTimerGetTimerID(xTimer);
    currentID++;

    if(currentID >= CYCLE_DUR) {
        currentID = 0;
    }

    vTimerSetTimerID(xTimer, currentID);
}
