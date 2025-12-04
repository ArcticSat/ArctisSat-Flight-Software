/*
 * healthAndSafety.h
 *
 *  Created on: Nov. 25, 2025
 *      Author: brend
 */

#ifndef INCLUDE_TASKS_HEALTHANDSAFETY_H_
#define INCLUDE_TASKS_HEALTHANDSAFETY_H_

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/queue.h>

#include "csp_server.h"


typedef enum errorType
{
    //Power errors
    ERR_POWER_LOST = 0x01,
    ERR_POWER_OVERVOLTAGE,
    ERR_POWER_UNDERVOLTAGE,
    ERR_POWER_OVERCURRENT,
    ERR_POWER_TEMPERATURE,
    ERR_POWER_MODE,
    WARN_POWER_COMMS_RESTORED,
    ERR_POWER_END,
    //ADCS errors
    ERR_ADCS_LOST,
    ERR_ADCS_NO_RESPONSE,
    ERR_SUN_SENSOR_FAIL,
    ERR_GYRO_FAIL,
    ERR_MAGNETOMETER_FAIL,
    ERR_ADCS_END,
    //Comms errors
    ERR_COMMS_LOST,
    ERR_COMMS_NO_RESPONSE,
    ERR_COMMS_END,
    //CDH errors
    ERR_CDH_END,
    //Payload errors
    ERR_PAYLOAD_END,
} errorType_t;

typedef enum severityLevel
{
    SEV_WARNING,
    SEV_LOW,
    SEV_MEDIUM,
    SEV_HIGH,
    SEV_CRITICAL,
} severityLevel_t;

typedef struct caughtError
{
    errorType_t errorType;
    severityLevel_t severity;
    uint8_t errorData[32];
} caughtError_t;



void logError(errorType_t type, severityLevel_t sev, uint8_t* data, size_t dataLen);


QueueHandle_t errorQueue;

void vHealthAndSafety();


#endif /* INCLUDE_TASKS_HEALTHANDSAFETY_H_ */
