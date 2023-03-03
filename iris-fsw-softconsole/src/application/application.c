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
#include "drivers/filesystem_driver.h"
#include "application/memory_manager.h"
#include "application/sc_deployment.h"

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

void InitMissionOperations(void)
{
	// Initialize the spacecraft's status
	int result_fs;
	result_fs = InitSpacecraftStatus();

    // Start up any tasks that depend on CSP, FS.
	uint8_t detumble_state;
	result_fs = getDetumblingStartupState(&detumble_state);
	if(result_fs == FS_OK && detumble_state == DETUMBLING_NOT_COMPLETE)
	{
		// Detumble mode
		vTaskResume(vDetumbleDriver_h);
	}
	else
	{
		// Normal operations
		InitNormalOperations();
	}
}

void InitNormalOperations(void)
{
	// Check deployment state
	uint8_t deployment_state;
	getDeploymentStartupState(&deployment_state);
	if(deployment_state == DPL_STATE_STOWED)
	{
//		InitiateSpacecraftDeployment();
	}

	// Resume tasks
	vTaskResume(vCanServer_h);
	vTaskResume(vTTTScheduler_h);
	vTaskResume(vFw_Update_Mgr_Task_h);
}

void HandleTm(csp_conn_t * conn, csp_packet_t * packet)
{
	int src = csp_conn_src(conn);
	switch(src){
		case PAYLOAD_CSP_ADDRESS:{
			HandlePayloadTlm(conn,packet);
			break;
		}
		default:{
			break;
		}
	}
}
