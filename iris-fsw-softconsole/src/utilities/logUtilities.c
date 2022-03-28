/*
 * logUtilities.c
 *
 *  Created on: Mar. 26, 2022
 *      Author: Jayden McKoy
 */

#include "utilities/logUtilities.h"


void logTask(time_tagged_task_t * ttt)
{
	char logMessage[256] = "";
	sprintf(logMessage,"%d,%d",ttt->request_code,ttt->parameter);
	Calendar_t time = ttt->time_tag;
	sprintf(logMessage,"%s,%d:%d:%d %d-%d-%d\n",logMessage,time.hour,time.minute,time.second,time.year,time.month,time.day);
	//sprintf(logMessage,"%s,%d:%d:%d %d-%d-%d\n",logMessage,time->hour,time->minute,time->second,time->year,time->month,time->day);
}

void createLogFiles(void)
{
	int result_fs = 1;
	lfs_file_t logfile = {0};

	// Create task log
	result_fs = fs_file_open( &logfile, TASK_LOG_FILE_PATH, LFS_O_CREAT);
	// Need to decide what to do on errors
	if(result_fs < 0) while(1){}
	// remember the storage is not updated until the file is closed successfully
	result_fs = fs_file_close( &logfile);
	if(result_fs < 0) while(1){}

	// Create telemetry log
	result_fs = fs_file_open( &logfile, TELEMETRY_LOG_FILE_PATH, LFS_O_CREAT);
	if(result_fs < 0) while(1){}
	result_fs = fs_file_close( &logfile);
	if(result_fs < 0) while(1){}

	// Create error log
	result_fs = fs_file_open( &logfile, ERROR_LOG_FILE_PATH, LFS_O_CREAT);
	if(result_fs < 0) while(1){}
	result_fs = fs_file_close( &logfile);
	if(result_fs < 0) while(1){}
}

void writeLog(char * logFileName, const void *buffer, lfs_size_t size)
{
	int result_fs = 1;
	lfs_file_t logfile = {0};
	// Open the file for write
	result_fs = fs_file_open( &logfile, logFileName, LFS_O_WRONLY);
	// Need to decide what to do on errors
	if(result_fs < 0) while(1){}
	// Write the data
	result_fs = fs_file_write( &logfile, buffer, size);
	if(result_fs < 0) while(1){}
	// Close the file to update the storage
	result_fs = fs_file_close( &logfile);
	if(result_fs < 0) while(1){}
}

void readLog(char * logFileName, void *buffer, lfs_size_t size)
{
	int result_fs = 1;
	lfs_file_t logfile = {0};
	// Open the file for write
	result_fs = fs_file_open( &logfile, logFileName, LFS_O_RDONLY);
	// Need to decide what to do on errors
	if(result_fs < 0) while(1){}
	// Read data into the buffer
	result_fs = fs_file_read( &logfile, buffer, size);
	if(result_fs < 0) while(1){}
	// Close the file
	result_fs = fs_file_close( &logfile);
	if(result_fs < 0) while(1){}
}
