#ifndef SCHEDULER_H
#define SCHEDULER_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Custom task scheduler. Uses a queue of time-tagged tasks to be executed at a given future time.
//
// History
// 2019-09-08 by Eric Kapilik
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "request_code.h"
#include "drivers/mss_rtc/mss_rtc.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description: Holds a desired execution time along with a request code.
//
//	time_tag: the desired future execution time of the request
//  request_code: the code of the predefined task to execute
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct {
	mss_rtc_calendar_t time_tag;
	uint8_t * parameters;
	request_code_t request_code;
} time_tagged_task_t;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Schedule a predefined task at a give future time.
//
// Parameters:
//	req - Predefined task request specified by request code.
//	time - Given future time to fulfill requested action at.
//
// Returns:
//  Integer - 0 if successfully scheduled task, error occurred otherwise.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int schedule_command(telemetryPacket_t * cmd_pkt);
int schedule_task_with_param(
		request_code_t req,
		uint8_t * params,
		mss_rtc_calendar_t time
		);
int schedule_task(
    request_code_t req,
	mss_rtc_calendar_t time
	);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Poll the queue of scheduled tasks and execute them at set time.
//
// Returns:
//  Void
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void vTestTaskScheduler(void *pvParameters);

void vTTT_Scheduler(void *pvParameters);
time_tagged_task_t * check_queue_for_task(mss_rtc_calendar_t time_now);

#endif // SCHEDULER_H
