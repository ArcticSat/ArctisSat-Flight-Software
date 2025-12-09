/*
 * healthAndSafety.c
 *
 *  Created on: Nov. 25, 2025
 *      Author: brend
 */

#include "tasks/healthAndSafety.h"

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>

#include "drivers/device/watchdog.h"
#include "tasks/telemetry.h"

char buf[32];
caughtError_t receivedError;
void vHealthAndSafety() {
  uint8_t loopCount = 0;
  printToTerminal("Health and Safety Task started.\n");
  for (;;) {
    handleErrors();
//    loopCount++;
    size_t freeHeapBytes = xPortGetFreeHeapSize();
    if (loopCount == 100) {
      syncFiles();
      loopCount = 0;
    }

    if (loopCount == 50) {
      // sprintf(buf, "H&S Loop %d: Free heap bytes: %lu\n", loopCount,
      //         (unsigned long)freeHeapBytes);
//      printToTerminal(buf);
      for (int i = 0; i < MAX_MONITORED_TASKS; i++) {
        // print stack high water mark for each monitored task
        if (monitoredTasks[i] != NULL) {
          UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(monitoredTasks[i]);

          volatile uint8_t j = 0;
          snprintf(buf, 32, "Task %d High Water Mark: %lu\n", i,
                   (unsigned long)highWaterMark);
          printToTerminal(buf);
        }
      }
    }
    service_WD();
    vTaskDelay(100);
    loopCount++;
  }
}

void handlePowerError(errorType_t errorType) {
  switch (errorType) {
    case ERR_POWER_LOST:
      printToTerminal("POWER COMMS LOST\n");
      logPowerTelem("POWER COMMS LOST\n", strlen("POWER COMMS LOST\n") + 1);
      // Add specific handling code here
      break;
    case WARN_POWER_COMMS_RESTORED:
      printToTerminal("POWER COMMS RESTORED\n");
      logPowerTelem("POWER COMMS RESTORED\n",
                    strlen("POWER COMMS RESTORED\n") + 1);
      // Add specific handling code here
      break;
    case ERR_POWER_OVERVOLTAGE:
      printToTerminal("POWER OVERVOLTAGE\n");
      logPowerTelem("POWER OVERVOLTAGE\n", strlen("POWER OVERVOLTAGE\n") + 1);
      // Add specific handling code here
      break;
    // Handle other power errors similarly
    default:
      printToTerminal("Unknown Power Error Type!\n");
      logPowerTelem("Unknown Power Error Type!\n",
                    strlen("Unknown Power Error Type!\n") + 1);
      break;
  }
}

void handleErrors() {
  caughtError_t receivedError;
  if (xQueueReceive(errorQueue, &receivedError, 0) == pdTRUE) {
    switch (receivedError.severity) {
      case SEV_WARNING:
        printToTerminal("! WARNING ! \n");
        break;
      case SEV_LOW:
        printToTerminal("!! LOW !! \n");
        break;
      case SEV_MEDIUM:
        printToTerminal("!!! MEDIUM !!! \n");
        break;
      case SEV_HIGH:
        printToTerminal("!!!! HIGH !!!! \n");
        break;
      case SEV_CRITICAL:
        printToTerminal("!!!!! CRITICAL !!!!! \n");
        break;
      default:
        printToTerminal("Unknown severity level! \n");
        break;
    }
    // Handle the error based on its type
    if (receivedError.errorType < ERR_POWER_END) {
      printToTerminal("POWER ERROR\n");
      handlePowerError(receivedError.errorType);
    } else if (receivedError.errorType < ERR_ADCS_END) {
      printToTerminal("ADCS ERROR\n");
      // Add specific handling for ADCS errors
    } else if (receivedError.errorType < ERR_COMMS_END) {
      printToTerminal("COMMUNICATIONS ERROR\n");
      // Add specific handling for communications errors
    } else {
      printToTerminal("ERROR\n");
    }
  }
}

void logError(errorType_t type, severityLevel_t severity, uint8_t* data,
              size_t dataLen) {
  caughtError_t newError;
  newError.errorType = type;
  newError.severity = severity;
  if (dataLen > 32) {
    dataLen = 32;  // Limit to 32 bytes
  }

  if (data != NULL) {
    memcpy(newError.errorData, data, dataLen);
  } else {
    memset(newError.errorData, 0, 32);
  }
  xQueueSendToBack(errorQueue, &newError, portMAX_DELAY);
}
