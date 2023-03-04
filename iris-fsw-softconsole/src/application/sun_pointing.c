/*
 * sun_pointing.c
 *
 *  Created on: Jan. 2, 2023
 *      Author: codyauch
 */

#include "application/sun_pointing.h"
#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include "timers.h"
#include "drivers/device/adcs_driver.h"

#define SAMPLE_SUN_SENSOR_DUR 1
#define SAMPLE_MAGS_CONTROL_DUR 4
#define CYCLE_DUR 10
#define SAMPLE_LOOPS 100


// PROTOTYPES:
void SunPointingP1( void );
void SunPointingP2( void );
void SunPointingP3( void );
void vHandleTimer(TimerHandle_t xTimer);


// Sun Pointing Task Timer
TimerHandle_t sunPointingTimer;

// Sun Pointing State Enum
typedef enum  {
    SAMPLE_SUN_SENSOR_GYROS = SAMPLE_SUN_SENSOR_DUR,
    SAMPLE_MAGS_CONTROL = SAMPLE_MAGS_CONTROL_DUR,
    COMMAND_TORQ_RODS = CYCLE_DUR
}SUN_POINTING_STATES;

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

    // Turn off torque rods
    setTorqueRodState(TORQUE_ROD_1, TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_2, TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_3, TR_STATE_OFF);

    uint8_t mag_readings[SAMPLE_LOOPS];
    uint8_t ss_readings[SAMPLE_LOOPS];

    // Loop SAMPLE_LOOP times
    for(int i=0; i<SAMPLE_LOOPS; i++)
    {
        /*
            - Do 1 mag measurement and conversion
            - Do 1 sun sensor measurement and conversion
        */
    }


}

void SunPointingP2( void ){}

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
