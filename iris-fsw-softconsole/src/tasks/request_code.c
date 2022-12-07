//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// MBSATS 2019-2020
//
// Repository:
//  ManitobaSat-Flight-Software
//
// File Description:
//  Request code function handling.
//
// History
// 2019-10-08 Eric Kapilik
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include "drivers/device/rtc/rtc_common.h"
#include "tasks/request_code.h"
#include "application/application.h"


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
// TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

void handle_request_with_param(TaskId_t req, uint8_t * params, Calendar_t time)
{
	telemetryPacket_t cmd_pkt = {0};
	cmd_pkt.telem_id = req;
	cmd_pkt.timestamp = time;
	cmd_pkt.data = params;
	// Dispatch command
	if(cmd_pkt.telem_id < CDH_COMMANDS_END)
	{
		HandleCdhCommand(&cmd_pkt);
	}
	else if(cmd_pkt.telem_id < POWER_COMMANDS_END)
	{
		HandlePowerCommand(&cmd_pkt);
	}
	else if(cmd_pkt.telem_id < PAYLOAD_COMMANDS_END)
	{

	}
	else if(cmd_pkt.telem_id < ADCS_COMMANDS_END)
	{
		HandleAdcsCommand(&cmd_pkt);
	}
	else
	{
		// TBC: error handling
	}
	// TBC: event logging
}

void handle_request(TaskId_t req,Calendar_t time){
	int temp = 0;
	switch(req){

		case(TASK_TAKE_IMAGE):{
		        uint8_t imgNum =1;
                telemetryPacket_t telemetry={0};
                telemetry.telem_id= PAYLOAD_FULL_IMAGE_CMD;
                telemetry.timestamp = time;
                telemetry.length = 1;
                telemetry.data = &imgNum;
                sendCommand(&telemetry, PAYLOAD_CSP_ADDRESS);

			break;
		}
		/* POWER FLATSAT TEST TTTs */
		case 1:{ // Get power to send 8 byte msg
			CANMessage_t cmd = {0};
			cmd.id = POW_TXID;
			cmd.dlc = 1;
			cmd.data[0] = 0;
			CAN_transmit_message(&cmd);
			break;
		}
		case 2:{ // Read temperature value command
	        uint8_t therm = 0; // ADC Channel 0
	        CANMessage_t cmd = {0};
	        cmd.id = POW_TXID;
	        cmd.dlc = 2;
	        cmd.data[0] = POWER_READ_TEMP_CMD;
	        cmd.data[1] = therm;
			CAN_transmit_message(&cmd);
			break;
		}
//		case(TEST_CODE_1):
//			temp = 456; // Do other stuff
//			break;
//		case(TEST_CODE_2):
//			temp = 789; // Do other stuff
//			break;
		default:
			return;
	}
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
