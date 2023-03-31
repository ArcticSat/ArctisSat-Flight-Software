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
#define NUM_SAMPLE_LOOPS_SUN_SENSOR			1
#define NUM_SAMPLE_LOOPS_GYROSCOPE			1
#define NUM_SAMPLE_LOOPS_MAGNETOMETER		1
// Back panel polling
#define BACKPANEL_SA_POLLING_LOOPS			5
// Telemetry rates
#define SUN_POINTING_TM_RATE 30
// Number of missed samples before restting ADCS
#define MISSED_SAMPLES_RESET_COUNT_MAX 		10*60
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
// Sun-pointing counters
uint32_t num_sun_pointing_cycles = 0;
uint32_t calibration_cycles = 0;
// Sun-pointing Task Timer
TimerHandle_t sunPointingTimer;
// Back panel solar array poll loop counter
uint8_t backpanel_loop_counter = 0;
// Sensor select
volatile enumSunSensor ss_x_select = SUN_SENSOR_PRIMARY_X; // TODO: confirm value
volatile enumSunSensor ss_z_select = SUN_SENSOR_PRIMARY_Z; // TODO: confirm value
volatile GyroId_t gyro_select = GYRO_1;
volatile MagnetometerId_t mag_select = MAGNETOMETER_1;
// Sensor raw data indices
volatile uint8_t gyro_x_lsb_idx = 2;
volatile uint8_t gyro_x_msb_idx = 3;
volatile uint8_t gyro_y_lsb_idx = 0;
volatile uint8_t gyro_y_msb_idx = 1;
volatile uint8_t gyro_z_lsb_idx = 4;
volatile uint8_t gyro_z_msb_idx = 5;
volatile gyro_x_sign_flip = false;
volatile gyro_y_sign_flip = true;
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
// Spacecraft is backwards, eclipse variables
volatile bool backwards = false;
volatile bool inEclipse = false;
volatile bool sunSensorsInRange = false;
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
// Missed samples counter
volatile bool all_samples_valid = false;
volatile uint32_t samples_missed_counter = 0;
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
	// Power the ADCS
    setLoadSwitch(LS_ADCS,SWITCH_ON);
    // Initialize timer
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
    	/*** Magnetometer calibration ***/
    	if(samples_missed_counter >= MISSED_SAMPLES_RESET_COUNT_MAX)
    	{
    		/*** Power cycle the ADCS ***/
    	    setLoadSwitch(LS_ADCS,SWITCH_OFF);
    	    vTaskDelay(100);
    	    setLoadSwitch(LS_ADCS,SWITCH_ON);
    	    vTaskDelay(100);
    		// Wait till next sampling phase
    		while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) != SAMPLE_SUN_SENSOR_GYROS);
    		// Reset counter
    		samples_missed_counter = 0;
    	}
    	/*** Sun-pointing tasks ***/
        while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == SAMPLE_SUN_SENSOR_GYROS)
        {
            SunPointingP1(); // 200ms
        }
        while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == SAMPLE_MAGS_CONTROL)
        {
            SunPointingP2(); // 300ms
        }
        while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == COMMAND_TORQ_RODS)
        {
            SunPointingP3(); // 500ms
        }
        // Update counter
        num_sun_pointing_cycles++;
        calibration_cycles++;
    }
}

void SunPointingP1( void )
{
	int i;

	/*** Turn off torque rods ***/
	setTorqueRodPwm(MAGNETORQUER_X,0);
	setTorqueRodPwm(MAGNETORQUER_Y,0);
	setTorqueRodPwm(MAGNETORQUER_Z,0);

	/*** Poll the back panels to check if Y- is facing the sun ***/
#ifdef BACKWARD_POLL_BACK_PANELS
	if(backpanel_loop_counter == BACKPANEL_SA_POLLING_LOOPS)
	{
		pollBackSolarPanels();
		backpanel_loop_counter = 0;
	}
	else
	{
		backpanel_loop_counter++;
	}
#endif

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
		vTaskDelay(10);
		if(ADCS_DRIVER_NO_ERROR != getSunSensorMeasurementsRaw(ss_x_buf[i])) continue;
		vTaskDelay(10);
		// Get Z-angle
		if(ADCS_DRIVER_NO_ERROR != sunSensorSelect(ss_z_select)) continue;
		vTaskDelay(10);
		if(ADCS_DRIVER_NO_ERROR != getSunSensorMeasurementsRaw(ss_z_buf[i])) continue;
		vTaskDelay(10);
		// Update valid samples
		ss_sample_valid[i] = true;
		num_valid_ss_samples++;
	}
	// Sample gyroscope
	for(i=0; i<NUM_SAMPLE_LOOPS_GYROSCOPE; i++)
	{
		if(ADCS_DRIVER_NO_ERROR != getGyroMeasurementsRaw(gyro_select,gyro_buf[i])) continue;
		vTaskDelay(10);
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
		vTaskDelay(10);
		if(ADCS_DRIVER_NO_ERROR != getMagnetometerMeasurementsRaw(mag_select, mag_buf[i])) continue;
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
			float x_angle, z_angle;
			// Convert X-angle
			x_angle = AngleDecompose(ss_x_buf[i],3);
			x_angle *= ss_x_sign_flip;
			ss_x_sum += sin(x_angle);
			// Convert Z-angle
			z_angle = AngleDecompose(ss_z_buf[i],3);
			z_angle *= ss_z_sign_flip;
			ss_z_sum += sin(z_angle);
		}
	}

	// Check that we have valid readings
#if defined(SIM_VALUES_1) || defined(SIM_VALUES_2)
	all_samples_valid = true;
#else
	all_samples_valid = 	(num_valid_ss_samples > 0) 	 \
						&&  (num_valid_gyro_samples > 0) \
						&&  (num_valid_mag_samples > 0);
#endif
	if(!all_samples_valid)
	{
		samples_missed_counter++;
	}
	else
	{
		// Convert gyroscope samples to radians per second
		gyro_x_sum = 0.0;
		gyro_y_sum = 0.0;
		gyro_z_sum = 0.0;
		for(i=0; i<NUM_SAMPLE_LOOPS_GYROSCOPE; i++)
		{
			if(gyro_sample_valid[i])
			{
				uint16_t rawGyro;
				int16_t rawGyroSigned;
				// Gyro X conversion and sum
				// NOTE: GYRO AXIS ALIGNMENT CORRECTED HERE
				rawGyro = 0;
				rawGyro |=  ((uint16_t) gyro_buf[i][gyro_x_lsb_idx]);
				rawGyro |= (((uint16_t) gyro_buf[i][gyro_x_msb_idx]) << 8);
				rawGyroSigned = 0;
				rawGyroSigned |=  ((int16_t) gyro_buf[i][gyro_x_lsb_idx]);
				rawGyroSigned |= (((int16_t) gyro_buf[i][gyro_x_msb_idx]) << 8);
				gyro_x_sum += convertGyroDataRawToRadiansPerSecond(rawGyro);
				// Gyro Y conversion and sum
				rawGyro = 0;
				rawGyro |=  ((uint16_t) gyro_buf[i][gyro_y_lsb_idx]);
				rawGyro |= (((uint16_t) gyro_buf[i][gyro_y_msb_idx]) << 8);
				gyro_y_sum += convertGyroDataRawToRadiansPerSecond(rawGyro);
				// Gyro Z conversion and sum
				rawGyro = 0;
				rawGyro |=  ((uint16_t) gyro_buf[i][gyro_z_lsb_idx]);
				rawGyro |= (((uint16_t) gyro_buf[i][gyro_z_msb_idx]) << 8);
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
				rawMag |=  ((uint16_t) mag_buf[i][0]);
				rawMag |= (((uint16_t) mag_buf[i][1]) << 8);
				mag_x_sum += convertMagDataRawToTeslas(rawMag);
				// Mag Y conversion and sum
				rawMag = 0;
				rawMag |=  ((uint16_t) mag_buf[i][2]);
				rawMag |= (((uint16_t) mag_buf[i][3]) << 8);
				mag_y_sum += convertMagDataRawToTeslas(rawMag);
				// Mag Z conversion and sum
				rawMag = 0;
				rawMag |=  ((uint16_t) mag_buf[i][4]);
				rawMag |= (((uint16_t) mag_buf[i][5]) << 8);
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
		else
		{
			ss_x = 0.0;
			ss_z = 0.0;
		}
		// Gyro average
		if(num_valid_gyro_samples > 0)
		{
			// NOTE: GYRO ORIENTATION CORRECTED HERE
	#ifndef GYRO_X_SIM_ENG_VALUE
			gyro_x = gyro_x_sum / num_valid_gyro_samples;
			if(gyro_x_sign_flip)
				gyro_x = -gyro_x;
	#else
			gyro_x = GYRO_X_SIM_ENG_VALUE;
	#endif
	#ifndef GYRO_Y_SIM_ENG_VALUE
			gyro_y = gyro_y_sum / num_valid_gyro_samples;
			if(gyro_y_sign_flip)
				gyro_y = -gyro_y;
	#else
			gyro_y = GYRO_Y_SIM_ENG_VALUE;
	#endif
	#ifndef GYRO_Z_SIM_ENG_VALUE
			gyro_z = gyro_z_sum / num_valid_gyro_samples;
	#else
			gyro_z = GYRO_Z_SIM_ENG_VALUE;
	#endif
		}
		else
		{
			gyro_x = 0.0;
			gyro_y = 0.0;
			gyro_z = 0.0;
		}
		// Mag average
		if(num_valid_mag_samples > 0)
		{
			// NOTE: MAG ORIENTATION CORRECTED HERE
	#ifndef MAG_X_SIM_ENG_VALUE
			mag_x = mag_x_sum / num_valid_mag_samples;
			mag_x += mag_x_offset;
	#else
			mag_x = MAG_X_SIM_ENG_VALUE;
	#endif
	#ifndef MAG_Y_SIM_ENG_VALUE
			mag_y = mag_y_sum / num_valid_mag_samples;
			mag_y = -mag_y;
			mag_y += mag_y_offset;
	#else
			mag_y = MAG_Y_SIM_ENG_VALUE;
	#endif
	#ifndef MAG_Z_SIM_ENG_VALUE
			mag_z = mag_z_sum / num_valid_mag_samples;
			mag_z += mag_z_offset;
	#else
			mag_z = MAG_Z_SIM_ENG_VALUE;
	#endif
		}
		else
		{
			mag_x = 0.0;
			mag_y = 0.0;
			mag_z = 0.0;
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
#ifdef BACKWARD_POLL_BACK_PANELS
		backwards = spacecraftIsBackwards();
#else
		backwards = sunSensorsOutOfRange(ss_x,ss_z);
		sunSensorsInRange = !sunSensorsOutOfRange(ss_x,ss_z);
#endif
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
	}

    /*** Wait ***/
    while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) == SAMPLE_MAGS_CONTROL);
}

void SunPointingP3( void )
{

#ifdef SUN_POINTING_DEBUG_TELEMETRY
	// Send telemetry
	if((SUN_POINTING_TM_RATE - 1) == sun_pointing_pkt_count)
	{
		SendSunPointingTelemetry()
		sun_pointing_pkt_count = 0;
	}
	else
	{
		sun_pointing_pkt_count++;
	}
#endif

	/*** Send the scaled commands to magnetorquers IF NOT IN ECLIPSE***/
#ifdef BACKWARD_POLL_BACK_PANELS
	inEclipse = !backwards && !sunSensorsInRange;
#else
	inEclipse = spacecraftInEclipse();
#endif
	if(!inEclipse && all_samples_valid)
	{
		// Set polarity
		setTorqueRodPolarity(MAGNETORQUER_X,polarity_x);
		setTorqueRodPolarity(MAGNETORQUER_Y,polarity_y);
		setTorqueRodPolarity(MAGNETORQUER_Z,polarity_z);
		// Set PWM
		setTorqueRodPwm(MAGNETORQUER_X,pwm_x);
		setTorqueRodPwm(MAGNETORQUER_Y,pwm_y);
		setTorqueRodPwm(MAGNETORQUER_Z,pwm_z);
	}

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

void SendSunPointingTelemetry(void)
{
	// Wait for P3
//	while(determineSunPointingState(pvTimerGetTimerID(sunPointingTimer)) != COMMAND_TORQ_RODS);
	// Format telemetry
	telemetryPacket_t tmpkt = {0};
	uint8_t data[58] = {0};
	tmpkt.telem_id = CDH_SUN_POINTING_ID;
	tmpkt.length = 59;
	tmpkt.data = data;
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
	// Eclipse
	memcpy(&data[58],(uint8_t *) &inEclipse, sizeof(uint8_t));
	// Send telemetry
	sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
}
