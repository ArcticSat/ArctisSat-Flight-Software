/*
 * sun_pointing.c
 *
 *  Created on: Jan. 2, 2023
 *      Author: codyauch
 */

#include "application/sun_pointing.h"

#define SAMPLE_SUN_SENSOR_DUR 1
#define SAMPLE_MAGS_CONTROL_DUR 4
#define CYCLE_DUR 10

// FreeRTOS
#include "timers.h"

// PROTOTYPES:
void SunPointingP1( void );
void SunPointingP2( void );
void SunPointingP3( void );
void vHandleTimer(TimerHandle_t xTimer);


// Sun Pointing Task Timer
TimerHandle_t sunPointingTimer;

// Sun Pointing State Enum
typedef enum SUN_POINTING_STATES {
    SAMPLE_SUN_SENSOR_GYROS = 0,
    SAMPLE_MAGS_CONTROL = 1,
    COMMAND_TORQ_RODS = 2
}

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

void SunPointingP1( void ){}

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
