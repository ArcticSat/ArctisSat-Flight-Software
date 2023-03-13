/*
 * eps_driver.c
 *
 *  Created on: Mar. 3, 2023
 *      Author: Jayden McKoy
 */

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "drivers/subsystems/eps_driver.h"
#include "drivers/protocol/can.h"
#include "tasks/telemetry.h"

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

void setLoadSwitch(uint8_t loadSwitchNumber, eSwitchState state)
{
	CANMessage_t cmd = {0};
	uint8_t cmd_id;
	switch(state)
	{
		case SWITCH_OFF:{
			cmd_id = POWER_SET_LOAD_OFF_CMD;
			break;
		}
		case SWITCH_ON:{
			cmd_id = POWER_SET_LOAD_ON_CMD;
			break;
		}
		default:{
			return;
		}
	}
	// Send CAN message
	int i;
	for(i=0; i < 10; i++){
		cmd.id = POW_TXID;
		cmd.dlc = 2;
		cmd.data[0] = cmd_id;
		cmd.data[1] = loadSwitchNumber;
		CAN_transmit_message(&cmd);
		vTaskDelay(50);
	}
}
