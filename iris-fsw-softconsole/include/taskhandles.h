#ifndef TASKHANDLES_H_
#define TASKHANDLES_H_

#include "task.h"


extern TaskHandle_t xUART0RxTaskToNotify;
extern TaskHandle_t vTTTScheduler_h;
extern TaskHandle_t vFw_Update_Mgr_Task_h;
extern TaskHandle_t vCanServer_h;
extern TaskHandle_t vCSP_Server_h;
//Debug Only:
extern TaskHandle_t vTaskSpinLEDs_h;

// Shared "monitored tasks" infra. This used to live in the (now removed) health-and-safety /
// watchdog module, but main.c still needs it to keep track of the demo tasks it creates.
#define MAX_MONITORED_TASKS 5
#define CSP_SERVER_TASK_INDEX   0
#define CSP_TX_TASK_INDEX       1
#define UART_HANDLER_TASK_INDEX 2
#define UART_TX_TASK_INDEX      3
#define UART_RX_TASK_INDEX      4

extern TaskHandle_t monitoredTasks[MAX_MONITORED_TASKS];

#endif
