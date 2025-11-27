/*
 * healthAndSafety.c
 *
 *  Created on: Nov. 25, 2025
 *      Author: brend
 */

 #include "tasks/healthAndSafety.h"
 #include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tasks/telemetry.h"

void handlePowerError(errorType_t errorType) {
    switch(errorType) {
        case ERR_POWER_LOST:
            printToTerminal("POWER COMMS LOST\n");
            logPowerTelem("POWER COMMS LOST\n", strlen("POWER COMMS LOST\n")+1);
            // Add specific handling code here
            break;
        case WARN_POWER_COMMS_RESTORED:
            printToTerminal("POWER COMMS RESTORED\n");
            logPowerTelem("POWER COMMS RESTORED\n", strlen("POWER COMMS RESTORED\n")+1);
            // Add specific handling code here
            break;
        case ERR_POWER_OVERVOLTAGE:
            printToTerminal("POWER OVERVOLTAGE\n");
            logPowerTelem("POWER OVERVOLTAGE\n", strlen("POWER OVERVOLTAGE\n")+1);
            // Add specific handling code here
            break;
        // Handle other power errors similarly
        default:
            printToTerminal("Unknown Power Error Type!\n");
            logPowerTelem("Unknown Power Error Type!\n", strlen("Unknown Power Error Type!\n")+1);
            break;
    }
}


void vHealthAndSafety() {
    caughtError_t receivedError;
    uint8_t loopCount = 0;
    char buf[32];
    printToTerminal("Health and Safety Task started.\n");
    for(;;) {
        size_t freeHeapBytes = xPortGetFreeHeapSize();

        if(xQueueReceive(errorQueue, &receivedError, 0) == pdTRUE) {
            printToTerminal("\n\n\n");
            switch(receivedError.severity) {
                case SEV_WARNING:
                    printToTerminal("! WARNING ! ");
                    break;
                case SEV_LOW:
                    printToTerminal("!! LOW !! ");
                    break;
                case SEV_MEDIUM:
                    printToTerminal("!!! MEDIUM !!! ");
                    break;
                case SEV_HIGH:
                    printToTerminal("!!!! HIGH !!!! ");
                    break;
                case SEV_CRITICAL:
                    printToTerminal("!!!!! CRITICAL !!!!! ");
                    break;
                default:
                    printToTerminal("Unknown severity level! ");
                    break;
            }
            // Handle the error based on its type
            if(receivedError.errorType < ERR_POWER_END) {
                printToTerminal("POWER ERROR: ");
                handlePowerError(receivedError.errorType);
            } else if(receivedError.errorType < ERR_ADCS_END) {
                printToTerminal("ADCS ERROR: ");
                // Add specific handling for ADCS errors
            } else if(receivedError.errorType < ERR_COMMS_END) {
                printToTerminal("COMMUNICATIONS ERROR: ");
                // Add specific handling for communications errors
            } else {
                printToTerminal("ERROR: ");
            }
            printToTerminal("\n\n\n");
        } else {
            if(loopCount % 10 == 0) {
                syncFiles();
            }
            sprintf(buf, "H&S Loop %d - Free Heap: %u bytes\n", loopCount++, (unsigned int)freeHeapBytes);
            printToTerminal(buf);
            vTaskDelay(1000);
        }
    }
}

void logError(errorType_t type, severityLevel_t severity, uint8_t* data, size_t dataLen) {
    caughtError_t newError;
    newError.errorType = type;
    newError.severity = severity;
    if(dataLen > 32) {
        dataLen = 32; // Limit to 32 bytes
    }
    
    if(data != NULL) {
        memcpy(newError.errorData, data, dataLen);
    } else {
        memset(newError.errorData, 0, 32);
    }
    xQueueSendToBack(errorQueue, &newError, portMAX_DELAY);
}


