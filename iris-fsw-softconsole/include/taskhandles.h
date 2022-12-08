#ifndef TASKHANDLES_H_
#define TASKHANDLES_H_

#include "task.h"

extern TaskHandle_t vTTTScheduler_h;
extern TaskHandle_t vFw_Update_Mgr_Task_h;
extern TaskHandle_t vCanServer_h;
extern TaskHandle_t vCSP_Server_h;
extern TaskHandle_t vTestWD_h;
//Debug Only:
extern TaskHandle_t vTaskSpinLEDs_h;

#endif
