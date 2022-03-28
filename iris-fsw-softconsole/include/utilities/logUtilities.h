/*
 * logUtilities.h
 *
 *  Created on: Mar. 26, 2022
 *      Author: Jayden McKoy
 */

#ifndef INCLUDE_UTILITIES_LOGUTILITIES_H_
#define INCLUDE_UTILITIES_LOGUTILITIES_H_

#include <littlefs/lfs.h>
#include "tasks/scheduler.h"

#define TASK_LOG_FILE_PATH	"cdh/taskLog.txt"
#define CDH_TELEMETRY_LOG_FILE_PATH	"cdh/telemetryLog.txt"
#define CDH_ERROR_LOG_FILE_PATH	"cdh/errorLog.txt"

void logTask(time_tagged_task_t * ttt);
void createLogFiles(void);
void writeLog(char * logFileName, const void *buffer, lfs_size_t size);
void readLog(char * logFileName, void *buffer, lfs_size_t size);

#endif /* INCLUDE_UTILITIES_LOGUTILITIES_H_ */
