/*
 * detumbling.c
 *
 *  Created on: Mar. 3, 2023
 *      Author: Jayden McKoy
 */

/*** DETUMBLE CODE REFERENCE:
 * repo:	https://github.com/IrisSat/IrisSat-Power-Software/commits/main
 * commit:	50c518598bcae95030d0def297654fe115ae8721
 * ***/

/***
 * TODO: check all warnings
 * ***/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "application/detumbling.h"
#include "main.h"
#include "drivers/device/adcs_driver.h"
#include "drivers/subsystems/eps_driver.h"
#include "application/memory_manager.h"
#include "application/application.h"
#ifdef DEBUG_DETUMBLING
#include "tasks/telemetry.h"
#endif

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Mission-level parameters
//#define MAX_DETUMBLE_TIME_SECONDS 4500
#define MAX_DETUMBLE_TIME_SECONDS 30
#define MIN_OMEGA 0.1
// Sampling parameters
#define NUM_MAG_SAMPLES 10
#define NUM_MAG_FINITE_DIFFERENCES (NUM_MAG_SAMPLES-1)

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum  {
    COLLECT_DATA = 1,
    CALC_EXEC_DIPOLE = 2,
    DETUMBLE_WAIT = 3
} eDetumbleStates;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
TaskHandle_t xHandleDetumble;
TimerHandle_t detumbleTimer;
// collectMagData
volatile int magData[NUM_MAG_SAMPLES][3] = {0};
// calculateExecuteDipole
float dipole[3];
int diffs[NUM_MAG_FINITE_DIFFERENCES][3] = {0};
volatile const float gain = -1.0;
volatile uint8_t polarity[3] = {0};
volatile uint8_t pwm[3] = {0};
// detumbleWait
uint16_t detumbling_cycles = 0;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#define MAG_LSB 0.25
#define GAUSS_TO_TESLA_CONVERSION 10000
int32_t convertMagData(uint16_t rawMag) {
    //Field range is +- 8 gauss and 16 bit accuracy therefore 1 LSB = 0.25mGauss

    return (-8) + (rawMag * MAG_LSB * GAUSS_TO_TESLA_CONVERSION); //needs to return 32-bit, as 8 Gauss = 80,000 Tesla > 2^16
}

#define DPS_TO_RPS 0.017448
int16_t convertGyroData(uint16_t rawGyro) {
    //+- 245 dps scale
    //Raw gyro data is 2's complement (signed) 16 bit values, should just be able to multiply by 2pi/360
    
    return ((int_16) rawGyro) * DPS_TO_RPS;
}

eDetumbleStates determineState(int inputID) {
	eDetumbleStates output = COLLECT_DATA;

    if(inputID <= 2) { //up to 200ms
        output = COLLECT_DATA;
    } else if(inputID <= 5) { //200 - 500 = 300ms
        output = CALC_EXEC_DIPOLE;
    } else { //1000 - 500 = 500ms
        output = DETUMBLE_WAIT;
    }

    return output;
}

bool detumblingComplete(void)
{
//    return detumbling_cycles > MAX_DETUMBLE_TIME_SECONDS;
    if(detumbling_cycles > MAX_DETUMBLE_TIME_SECONDS)
        return true;
    int i;
    for(i=0; i < 3; i++){
        if(dipole[i] > MAX_DIPOLE)
            return false;
    }
    return true;
}

void collectMagData(void)
{
    // Get measurements
    uint8_t rawMagData[6] = {0};
    uint16_t tempMagData[3];
    int i;
    int magDataIndex = 0;
    while(determineState(pvTimerGetTimerID(detumbleTimer)) == COLLECT_DATA && magDataIndex < NUM_MAG_SAMPLES) {
        // Get measurements
        getMagnetometerMeasurements(1, rawMagData);
        // TODO: capture sample time (store in a public variable)

        tempMagData[0] = (rawMagData[0] << 8) | rawMagData[1];
        tempMagData[1] = (rawMagData[2] << 8) | rawMagData[3];
        tempMagData[2] = (rawMagData[4] << 8) | rawMagData[5];

        for(i = 0; i < 3; i++) {
            magData[magDataIndex][i] = tempMagData[i];
        }
        magDataIndex++;
    }
    // Wait
    while(determineState(pvTimerGetTimerID(detumbleTimer)) == COLLECT_DATA);
}

void calculateExecuteDipole(void)
{
    int totalMeasurements = 0;
    int i, j;
    float voltage = 0.0;
    while(determineState(pvTimerGetTimerID(detumbleTimer)) == CALC_EXEC_DIPOLE) {
        /*** Calculate Dipole ***/
        // Calculate finite different of magnetometer data
        for(i = 0; i < NUM_MAG_FINITE_DIFFERENCES; i++) {
            if(magData[i][0] != 0 && magData[i+1][0] != 0) {
                for(j = 0; j < 3; j++) {
                    diffs[i][j] = magData[i+1][j] - magData[i][j];
                    // TODO: divide by delta t
                }
                totalMeasurements++;
            }
        }

        for(i = 0; i < totalMeasurements; i++) {
            dipole[0] += diffs[i][0];
            dipole[1] += diffs[i][1];
            dipole[2] += diffs[i][2];
        }
        // Calculate dipole
        dipole[0] = dipole[0] / totalMeasurements;
        dipole[1] = dipole[1] / totalMeasurements;
        dipole[2] = dipole[2] / totalMeasurements;
        // Calculate polarity, pwm
        for(i=0; i < 3; i++){
            // Gain
            dipole[i] *= gain;
            // Voltage
            voltage = dipoleToVoltage(dipole[i]);
            // Polarity
            polarity[i] = ((voltage >= 0) ? 1 : 0);
            // PWM
            pwm[i] = (uint8_t)( ((polarity[1]-1) * voltage ) * ( MAX_VOLTAGE / MAX_PWM ));
        }
        /*** Execute Dipole ***/
        // Set polarity
        setTorqueRodPolarity(TORQUE_ROD_1,polarity[0]);
        setTorqueRodPolarity(TORQUE_ROD_2,polarity[1]);
        setTorqueRodPolarity(TORQUE_ROD_3,polarity[2]);
        // Set PWM
        setTorqueRodPwm(TORQUE_ROD_1,pwm[0]);
        setTorqueRodPwm(TORQUE_ROD_2,pwm[1]);
        setTorqueRodPwm(TORQUE_ROD_3,pwm[2]);
        // Turn torque rods on
        setTorqueRodState(TORQUE_ROD_1,TR_STATE_ON);
        setTorqueRodState(TORQUE_ROD_2,TR_STATE_ON);
        setTorqueRodState(TORQUE_ROD_3,TR_STATE_ON);

        // TODO: send some telemetry
#ifdef DEBUG_DETUMBLING
        /*** Format data ***/
        int len = sizeof(dipole);
        uint8_t data[20] = {0};
        // Dipoles
        memcpy(data,dipole,3*sizeof(float));
        // Polarities
        memcpy(&data[6],polarity,3*sizeof(uint8_t));
        // PWMs
        memcpy(&data[9],pwm,3*sizeof(uint8_t));
        /*** Send packet ***/
		telemetryPacket_t tmpkt = {0};
		tmpkt.telem_id = CDH_DETUMBLING_TM_ID;
		sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
#endif


        while(determineState(pvTimerGetTimerID(detumbleTimer)) == CALC_EXEC_DIPOLE); // wait after calc
    }
}

void detumbleWait(void)
{
    // Turn torque rods off
    setTorqueRodState(TORQUE_ROD_1,TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_2,TR_STATE_OFF);
    setTorqueRodState(TORQUE_ROD_3,TR_STATE_OFF);
    // Detumbling complete??
    if(detumblingComplete())
    {
    	// TODO: exit detumbling
    	setDetumblingStartupState(DETUMBLING_COMPLETE);
        InitNormalOperations();
        while(1);
    }
    detumbling_cycles++;

    // Wait
    while(determineState(pvTimerGetTimerID(detumbleTimer)) == DETUMBLE_WAIT); // wait
}

void vHandleDetumbleTimer(TimerHandle_t xTimer)
{
    int currentID = pvTimerGetTimerID(xTimer);
    currentID++;

    if(currentID > 10) {
        currentID = 1;
    }

    vTimerSetTimerID(xTimer, currentID);
}

void vDetumbleDriver(void)
{
	// Power the ADCS
    setLoadSwitch(LS_ADCS,SWITCH_ON);
    // Initialize timer
    detumbleTimer = xTimerCreate("detumbleTimer", pdMS_TO_TICKS(100), pdTRUE, ( void * ) COLLECT_DATA, vHandleDetumbleTimer);
//    StaticTimer_t timerSpace;
//    TimerHandle_t detumbleTimerStatic = xTimerCreateStatic("detumbleTimer", pdMS_TO_TICKS(333), pdTRUE, ( void * ) COLLECT_DATA, vHandleTimer, &timerSpace);
    if(detumbleTimer == NULL)
    {
    	// TODO: error handling
//        InitNormalOps();
    	setDetumblingStartupState(DETUMBLING_COMPLETE);
        while(1)
        {
        	vTaskDelay(10000);
        }
    }
    else
    {
        xTimerStart(detumbleTimer,portMAX_DELAY);
    }
    for(;;) {
        while(determineState(pvTimerGetTimerID(detumbleTimer)) == COLLECT_DATA) {
            collectMagData();
        }

        while(determineState(pvTimerGetTimerID(detumbleTimer)) == CALC_EXEC_DIPOLE) {
            calculateExecuteDipole();
        }

        while(determineState(pvTimerGetTimerID(detumbleTimer)) == DETUMBLE_WAIT) {
            detumbleWait();
        }
    }
}
