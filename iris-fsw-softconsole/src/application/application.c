/*
 * application.c
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "application/application.h"
#include "taskhandles.h"
#include "main.h"
#include "drivers/filesystem_driver.h"
#include "application/memory_manager.h"
#include "application/sc_deployment.h"

#include "tasks/telemetry.h"

#include "application/cdh.h"
#include "application/eps.h"
#include "application/payload.h"
#include "application/adcs.h"

#include "FreeRTOS.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

int canWrite = 0;
char buf[32];

void vMissionLoop() {
    int seconds = 0;
    uint8_t remainingTasks;
    telemReadFlag = 0;
    printToTerminal("Mission Operations Loop started.\n");
    uint8_t loopCount = 0;
    for(;;) {
        Calendar_t currTime;
        //Check time tagged tasks
        scheduleTTTFromQueue();
        executeTTT();
        //Handle mission states
        //
        //
        //
        if(loopCount % 20 == 0) {
            ds1393_read_time(&currTime);
            sprintf(buf, "System time: %02u:%02u:%02u\n", currTime.hour, currTime.minute, currTime.second);
            printToTerminal(buf);
            loopCount = 0;
        }
        vTaskDelay(500);
        loopCount++;
    }
}


void scheduleTimeTaggedTask(timeTaggedTask_t* task)
{
    xQueueSendToBack(taskQueue, task, portMAX_DELAY);
}

void scheduleTTTFromQueue() {
    static timeTaggedTask_t task;
    int result = 0;
    if(xQueueReceive(taskQueue, &task, 0) == pdTRUE) {
        printToTerminal("\nScheduling time-tagged task...\n");
        fs_file_seek(&timeTaggedTaskFile, 0, LFS_SEEK_END);
        result = fs_file_write(&timeTaggedTaskFile, &task, sizeof(task));
        if(result < 0) {
            printToTerminal("Error writing time-tagged task to file\n");
            return;
        }
        fs_file_sync(&timeTaggedTaskFile);
        printToTerminal("Task scheduled.\n");
    }
    fs_file_rewind(&timeTaggedTaskFile);
}

void executeTTT() {
    Calendar_t currTime;
    ds1393_read_time(&currTime);
    int remainingTasks = 0;

    static timeTaggedTask_t task;
    int result;
    while(1) {
    result = fs_file_read(&timeTaggedTaskFile, &task, sizeof(task));
    if (result <= 0) {
        if(fs_file_size(&timeTaggedTaskFile) > 0) {
            printToTerminal("End of time-tagged tasks\n");
        }
        if(!remainingTasks && fs_file_size(&timeTaggedTaskFile) > 0) {
            printToTerminal("No remaining tasks. Formatting file\n");
            fs_file_rewind(&timeTaggedTaskFile);
            fs_file_truncate(&timeTaggedTaskFile, 0);
            fs_file_sync(&timeTaggedTaskFile);
        }
        break;
    } else {
        printToTerminal("Found time tagged task... ");
        sprintf(buf, "Task time: %u %u %u ", task.executionTime.hour, task.executionTime.minute, task.executionTime.second);
        printToTerminal(buf);
        if(task.executionTime.second >= (uint8_t) 60) {
            continue;
        }
        if(compare_time(&currTime, &task.executionTime) >= 0) {
            printToTerminal("Executing time-tagged task!\n");
            //Execute task
            //Mark task as executed by setting time to invalid value
            task.executionTime.second = 255;
            //Seek back to the position of this task
            fs_file_seek(&timeTaggedTaskFile, -((lfs_soff_t)sizeof(task)), LFS_SEEK_CUR);
            fs_file_write(&timeTaggedTaskFile, &task, sizeof(task));
            fs_file_sync(&timeTaggedTaskFile);

            printToTerminal("Task marked as executed.\n");
        } else {
            printToTerminal("Task not due yet.\n");
            remainingTasks = 1;
        }
    }
    }
}

void InitMissionOperations(void)
{

}

void InitNormalOperations(void)
{

}

void HandleTm(csp_conn_t * conn, csp_packet_t * packet)
{

}
