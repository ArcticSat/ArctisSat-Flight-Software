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
#include <FreeRTOS-Kernel/include/task.h>

#include "csp_server.h"

QueueHandle_t errorQueue;

#define MAX_MONITORED_TASKS 10
#define CSP_SERVER_TASK_INDEX 0
#define CSP_TX_TASK_INDEX 1
#define UART_HANDLER_TASK_INDEX 2
#define UART_TX_TASK_INDEX 3
#define UART_RX_TASK_INDEX 4
#define ADCS_DRIVER_TASK_INDEX 5
#define POWER_DRIVER_TASK_INDEX 6
#define TELEMETRY_TASK_INDEX 7
#define MISSION_TASK_INDEX 8
#define HEALTH_AND_SAFETY_TASK_INDEX 9

uint8_t powerStatus;
uint8_t adcsStatus;
uint8_t commsStatus;


//array of task handles
TaskHandle_t monitoredTasks[MAX_MONITORED_TASKS];


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
    WARN_ADCS_COMMS_RESTORED,
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


/*This function logs an error to the health and safety system.
Parameters:
    type - The type of error (from errorType_t enum)
    sev - The severity level of the error (from severityLevel_t enum)
    data - Pointer to additional data about the error
    dataLen - Length of the additional data
*/
void logError(errorType_t type, severityLevel_t sev, uint8_t* data, size_t dataLen);


/*
FreeRTOS Task for Health and Safety monitoring. Handles error processing and system health checks.
*/
void vHealthAndSafety();


#endif /* INCLUDE_TASKS_HEALTHANDSAFETY_H_ */
