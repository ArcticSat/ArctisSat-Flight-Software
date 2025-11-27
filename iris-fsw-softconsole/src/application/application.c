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
    timeTaggedTask_t task;
    int seconds = 0;
    uint8_t remainingTasks;
    printToTerminal("Mission Operations Loop started.\n");
    for(;;) {
        vTaskDelay(1000);
        printToTerminal(".");
        int result;
        canWrite = 0;
        remainingTasks = 0;
        if(xQueueReceive(taskQueue, &task, 0) == pdTRUE) {
            printToTerminal("\nScheduling time-tagged task...\n");
            //Append task to file
            fs_file_seek(&timeTaggedTaskFile, 0, LFS_SEEK_END);
            fs_file_write(&timeTaggedTaskFile, &task, sizeof(task));
            fs_file_sync(&timeTaggedTaskFile);
            printToTerminal("Task scheduled.\n");
        }
        fs_file_rewind(&timeTaggedTaskFile);
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
                sprintf(buf, "Task seconds: %u ", task.executionTime.second);
                printToTerminal(buf);
                if(task.executionTime.second == 255) {
                    printToTerminal("Invalid task, skipping...\n");
                    continue;
                }
                if(task.executionTime.second <= seconds) {
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
        seconds++;
        canWrite = 1;
        //Gather telemetry
        //Execute ground commands
        //Maintain health of subsystems
        //Check time tagged tasks
        //Handle mission states
    }
}


void scheduleTimeTaggedTask(timeTaggedTask_t* task)
{
    xQueueSendToBack(taskQueue, task, portMAX_DELAY);
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
