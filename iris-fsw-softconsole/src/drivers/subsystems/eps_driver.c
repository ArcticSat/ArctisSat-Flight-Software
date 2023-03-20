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

#include "FreeRTOS.h"
#include "task.h"

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
volatile float sa_current_eclipse_threshold = 10.0;
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

void pollBackSolarPanels(void)
{
	uint8_t i;
	CANMessage_t cmd = {0};
	cmd.id = POW_TXID;
	// Identifier for back panels: multiple MSBV polls
	cmd.dlc = 1;
	cmd.id = POWER_READ_MSB_VOLTAGE_ID;
	for(i=0; i < NUM_MSVB_POLLS_FOR_BACK_SA_PANELS; i++)
	{
		CAN_transmit_message(&cmd);
		vTaskDelay(10);
	}
	// Poll back bottom panel
	cmd.dlc = 2;
	cmd.data[0] = POWER_READ_SOLAR_CURRENT_CMD;
	cmd.data[1] = SA_BACK_STATIONARY_BOTTOM;
	CAN_transmit_message(&cmd);
	vTaskDelay(10);
	// Identifier for back panels: multiple MSBV polls
	cmd.dlc = 1;
	cmd.id = POWER_READ_MSB_VOLTAGE_ID;
	for(i=0; i < NUM_MSVB_POLLS_FOR_BACK_SA_PANELS; i++)
	{
		CAN_transmit_message(&cmd);
		vTaskDelay(10);
	}
	// Back top
	cmd.dlc = 2;
	cmd.data[0] = POWER_READ_SOLAR_CURRENT_CMD;
	cmd.data[1] = SA_BACK_STATIONARY_TOP;
	CAN_transmit_message(&cmd);
	// TODO: push messages through??
}
