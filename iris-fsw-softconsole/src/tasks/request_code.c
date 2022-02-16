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

void handle_request_with_param(cdhTask_t req, uint8_t param, Calendar_t time)
{
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
		case TASK_POWER_READ_TEMP:{ // Read temperature value command
			CANMessage_t cmd = {0};
			cmd.id = POW_TXID;
			cmd.dlc = 2;
			cmd.data[0] = POWER_READ_TEMP_CMD;
			cmd.data[1] = param;
			CAN_transmit_message(&cmd);
			break;
		}
		case TASK_POWER_READ_SOLAR_CURRENT:{ // Read solar current command
			CANMessage_t cmd = {0};
			cmd.id = POW_TXID;
			cmd.dlc = 2;
			cmd.data[0] = POWER_READ_TEMP_CMD;
			cmd.data[1] = param;
			cmd.data[0] = POWER_READ_SOLAR_CURRENT_CMD;
			cmd.data[1] = param;
			CAN_transmit_message(&cmd);
			break;
		}
		case TASK_POWER_READ_LOAD_CURRENT:{ // Read solar current command
			CANMessage_t cmd = {0};
			cmd.id = POW_TXID;
			cmd.dlc = 2;
			cmd.data[0] = POWER_READ_LOAD_CURRENT_CMD;
			cmd.data[1] = param;
			CAN_transmit_message(&cmd);
			break;
		}
		case TASK_POWER_READ_MSB_VOLTAGE:{ // Read solar current command
			CANMessage_t cmd = {0};
			cmd.id = POW_TXID;
			cmd.dlc = 1;
			cmd.data[0] = POWER_READ_MSB_VOLTAGE_CMD;
			CAN_transmit_message(&cmd);
			break;
		}
		case TASK_POWER_SET_MODE:{ // Read solar current command
			CANMessage_t cmd = {0};
			cmd.id = POW_TXID;
			cmd.dlc = 1;
			cmd.data[0] = POWER_SET_POW_MODE_CMD;
			CAN_transmit_message(&cmd);
			break;
		}
		default:
			break;
	}
}

void handle_request(cdhTask_t req,Calendar_t time){
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
