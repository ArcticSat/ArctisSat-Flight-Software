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
// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS Libraries
#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>

// RTC Libraries
#include "drivers/device/rtc/rtc_common.h"
#include "drivers/mss_rtc/mss_rtc.h"

// User Libraries
#include "tasks/scheduler.h"
#include "tasks/priority_queue.h"
#include "drivers/device/rtc/rtc_time.h"
#include "tasks/request_code.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SCHEDULER_TASK_DELAY_MS				(1500)	// The delay time of each task cycle in ms.

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static Node* priority_queue_handler;
static mss_rtc_calendar_t p_rtc_calendar; // Current time holder

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void init_TaskScheduler(void){
	//TODO: Ensure Internal RTC is synchronized with External RTC.

	//set up static variables
	priority_queue_handler = NULL;
}

time_tagged_task_t * check_queue_for_task(mss_rtc_calendar_t time_now){
	request_code_t result;

	time_tagged_task_t * temp = {0};
	if(isEmpty(&priority_queue_handler)){
		time_tagged_task_t temp1 = {0};
		temp1.request_code = INVALID_REQUEST_CODE;
		temp = &temp1;
		return temp; // Queue is currently empty
	}

	temp = peek(&priority_queue_handler); // look at task which has the soonest time

	if(compare_time(&(temp->time_tag), &time_now) <= 0){ //if soonest task is scheduled for current time or sooner
		result = temp->request_code;  // Save request code of the current task
		pop(&priority_queue_handler); //We have dealt with the task, so remove the task from the queue.
	}
	else{
		temp->request_code = INVALID_REQUEST_CODE;
		result = INVALID_REQUEST_CODE; // No task for this time
	}

	return temp;
}

request_code_t check_queue(mss_rtc_calendar_t time_now){
	request_code_t result;

	time_tagged_task_t* temp;
	if(isEmpty(&priority_queue_handler)){
		return INVALID_REQUEST_CODE; // Queue is currently empty
	}

	temp = peek(&priority_queue_handler); // look at task which has the soonest time

	if(compare_time(&(temp->time_tag), &time_now) <= 0){ //if soonest task is scheduled for current time or sooner
		result = temp->request_code;  // Save request code of the current task
		pop(&priority_queue_handler); //We have dealt with the task, so remove the task from the queue.
	}
	else{
		result = INVALID_REQUEST_CODE; // No task for this time
	}

	return result;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void vTestTaskScheduler(void *pvParameters){
	request_code_t request;
	init_TaskScheduler();

	//WARNING: DO NOT USE THIS TASK OUTSIDE OF TESTING. THE RTC IS RESET to 0.
	//IN ACTUAL OPERATION IT SHOULD BE SYNCED TO THE EXTERNAL CLOCK.

	MSS_RTC_stop(); // Stop the RTC
	MSS_RTC_reset_counter(); // Set RTC to 0

	static BaseType_t rslt; // Variable to hold result of various functions
	static time_tagged_task_t task_buf; // Buffer variable to hold a task

	MSS_RTC_get_calendar_count(&p_rtc_calendar); // get current time

	// add 60s to current time
	mss_rtc_calendar_t p_rtc_cal_1 = p_rtc_calendar;
	p_rtc_cal_1.minute += 1;
	schedule_task(TEST_CODE_1, p_rtc_cal_1); // create task to be executed in 60s

	// add 90s to current time
	mss_rtc_calendar_t p_rtc_cal_2 = p_rtc_calendar;
	p_rtc_cal_2.second += 30;
	p_rtc_cal_2.minute += 1;
	schedule_task(TEST_CODE_2, p_rtc_cal_2); // create task to be executed in 90s

	// add 30s to current time
	mss_rtc_calendar_t p_rtc_cal_0 = p_rtc_calendar;
	p_rtc_cal_0.second += 30;
	schedule_task(TEST_CODE_0, p_rtc_cal_0); // create task to be executed in 30s

	MSS_RTC_start(); // Start RTC

	for( ;; ) {
		MSS_RTC_get_calendar_count(&p_rtc_calendar); // get current time

		request = check_queue(p_rtc_calendar); // check queue for tasks at this current time

		if(request != INVALID_REQUEST_CODE){
			handle_request(request,p_rtc_calendar);
		}

		vTaskDelay(pdMS_TO_TICKS(SCHEDULER_TASK_DELAY_MS));
	}
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void vTTT_Scheduler(void *pvParameters){
    request_code_t request;
    time_tagged_task_t * task;
    init_TaskScheduler();

//    static BaseType_t rslt; // Variable to hold result of various functions
//    static time_tagged_task_t task_buf; // Buffer variable to hold a task
//
//    MSS_RTC_get_calendar_count(&p_rtc_calendar); // get current time
//
//    // add 60s to current time
//    mss_rtc_calendar_t p_rtc_cal_1 = p_rtc_calendar;
//    p_rtc_cal_1.minute += 1;
//    schedule_task(TEST_CODE_1, p_rtc_cal_1); // create task to be executed in 60s
//
//    // add 90s to current time
//    mss_rtc_calendar_t p_rtc_cal_2 = p_rtc_calendar;
//    p_rtc_cal_2.second += 30;
//    p_rtc_cal_2.minute += 1;
//    schedule_task(TEST_CODE_2, p_rtc_cal_2); // create task to be executed in 90s
//
//    // add 30s to current time
//    mss_rtc_calendar_t p_rtc_cal_0 = p_rtc_calendar;
//    p_rtc_cal_0.second += 30;
//    schedule_task(TEST_CODE_0, p_rtc_cal_0); // create task to be executed in 30s
//
//    MSS_RTC_start(); // Start RTC

    for( ;; ) {
        MSS_RTC_get_calendar_count(&p_rtc_calendar); // get current time

//        request = check_queue(p_rtc_calendar); // check queue for tasks at this current time
        task = check_queue_for_task(p_rtc_calendar);
        request = task->request_code;

        if(request != INVALID_REQUEST_CODE){
//            handle_request(request,p_rtc_calendar);
        	uint8_t * params = task->parameters;
            handle_request_with_param(request,params,p_rtc_calendar);
        }

        vTaskDelay(pdMS_TO_TICKS(SCHEDULER_TASK_DELAY_MS));
    }
}

int schedule_command(telemetryPacket_t * cmd_pkt)
{
	// construct time_tagged_task_t w/ given values
	time_tagged_task_t* pvTask = malloc(sizeof(time_tagged_task_t)); // private task to be initialized with parameters and copied into Queue.
	if(pvTask == NULL){
		return -1; // memory allocation error
	}

	pvTask->request_code = cmd_pkt->telem_id;
	pvTask->parameters = cmd_pkt->data;
	pvTask->time_tag = cmd_pkt->timestamp;
	unsigned long priority = CALENDAR_TO_LONG(&cmd_pkt->timestamp);

	if(isEmpty(&priority_queue_handler)){ // Check if priority queue is empty
		priority_queue_handler = newNode((void*) pvTask, priority); //	set priority queue = new Node(data)
	}
	else{
		push(&priority_queue_handler, (void*) pvTask, priority); // just put task into priority queue
	}

	return 0; // success
}

int schedule_task_with_param(request_code_t req, uint8_t * params, mss_rtc_calendar_t time){
	// construct time_tagged_task_t w/ given values
	time_tagged_task_t* pvTask = malloc(sizeof(time_tagged_task_t)); // private task to be initialized with parameters and copied into Queue.
	if(pvTask == NULL){
		return -1; // memory allocation error
	}

	pvTask->request_code = req;
	pvTask->parameters = params;
	pvTask->time_tag = time;
	unsigned long priority = CALENDAR_TO_LONG(&time);

	if(isEmpty(&priority_queue_handler)){ // Check if priority queue is empty
		priority_queue_handler = newNode((void*) pvTask, priority); //	set priority queue = new Node(data)
	}
	else{
		push(&priority_queue_handler, (void*) pvTask, priority); // just put task into priority queue
	}

	return 0; // success
}

int schedule_task(request_code_t req, mss_rtc_calendar_t time){
	// construct time_tagged_task_t w/ given values
	time_tagged_task_t* pvTask = malloc(sizeof(time_tagged_task_t)); // private task to be initialized with parameters and copied into Queue.
	if(pvTask == NULL){
		return -1; // memory allocation error
	}

	pvTask->request_code = req;
	pvTask->time_tag = time;
	unsigned long priority = CALENDAR_TO_LONG(&time);

	if(isEmpty(&priority_queue_handler)){ // Check if priority queue is empty
		priority_queue_handler = newNode((void*) pvTask, priority); //	set priority queue = new Node(data)
	}
	else{
		push(&priority_queue_handler, (void*) pvTask, priority); // just put task into priority queue
	}

	return 0; // success
}
