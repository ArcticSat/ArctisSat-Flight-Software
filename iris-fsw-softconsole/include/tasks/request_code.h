#ifndef REQUEST_CODE_H
#define REQUEST_CODE_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Predefined task codes for well-defined tasks.
//
// History
// 2019-09-08 by Eric Kapilik
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "tasks/telemetry.h"    //Request Codes are here.
// FOR FLATSAT POWER INTERFACING
#include "drivers/protocol/can.h"
//#include "application/eps.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
/*-------------------------------------------------------------------------*//**
  The request_code_t enumeration is used as the reqeust code parameter for
  the schedule_task() function in the scheduler.c module
  to specify what predefined requested action to take.
 */
typedef enum {
	INVALID_REQUEST_CODE = -1,
	TEST_CODE_0 = 1000,
	TEST_CODE_1 = 1001,
	TEST_CODE_2 = 1002
} request_code_t;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

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
//  Execute code for a predefined request_code_t
//
// Parameters:
//	req - Predefined task request specified by request code.
//
// Returns:
//  VOID
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void handle_request(TaskId_t req,Calendar_t time);
void handle_request_with_param(TaskId_t req, uint8_t * params, Calendar_t time);

#endif // REQUEST_CODE_H
