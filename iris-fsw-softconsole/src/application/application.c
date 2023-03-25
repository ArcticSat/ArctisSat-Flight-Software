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
#include "main.h"
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
	// Initialize the memory manager
	init_memory_manager();
	// Initialize the spacecraft's status
	int result_fs;
	result_fs = InitSpacecraftStatus();
	// Check deployment state
	uint8_t deployment_state;
	getDeploymentStartupState(&deployment_state);
	if(deployment_state == DPL_STATE_STOWED)
	{
//		InitiateSpacecraftDeployment();
		setDeploymentStartupState(DPL_STATE_DEPLOYED);
	}

	uint8_t rebootReason=REBOOT_UNKNOWN;
	getLastRebootReason(&rebootReason);
	if((rebootReason & REBOOT_OTA_UPDATE) && rebootReason != 0xFF){

	    //We need to powercycle the cdh for the FS, coreSPI etc to work.
	    //But first, SET the reboot reason to "request" so we don't boot loop.
	    int res = setLastRebootReason(REBOOT_POWER_REQUEST);

	    //Only request if the last step passes.
	    if(res == MEM_MGR_OK){

	        //Send command to power to reset.
	        //printf("Power cycled CDH\n ");
	    	//powercycleCDH();
	    	resetLoadSwitch(LS_CDH);

	        //Shouldn't ever get here, if we do... just continue on, we will have to manually fix.
	        printf("cdh request pwr cycle failed!\n");
	    }

	}

	int result;
	ScStatus_t sc_status;
	result = getScStatus(&sc_status);
	// Format data
	uint8_t buf[sizeof(result)+SC_STATUS_SIZE_BYTES] = {0};
	memcpy(buf,&result,sizeof(result));
	memcpy(&buf[sizeof(result)],&sc_status,sizeof(SC_STATUS_SIZE_BYTES));
	// Send telemetry packet
	telemetryPacket_t tmpkt = {0};
	tmpkt.telem_id = CDH_SPACECRAFT_STATUS_ID;
	tmpkt.length = sizeof(result)+SC_STATUS_SIZE_BYTES;
	tmpkt.data = buf;
	sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);

	//Send the results of the prvSetupHardware.
    telemetryPacket_t hwStatPkt = {0};
    hwStatPkt.telem_id = CDH_HW_STATUS_ID;
    hwStatPkt.length = sizeof(HardwareCheck_t);
    hwStatPkt.data = (uint8_t*)&setupHardwareStatus;
    sendTelemetryAddr(&hwStatPkt, GROUND_CSP_ADDRESS);


//	int ping_result = -1;
//	ping_result = csp_ping(10, 5000, 50, 0);
//	int x = 7;


	// Resume tasks
//	vTaskResume(vSunPointing_h);
//	vTaskResume(vCanServer_h);
//	vTaskResume(vTTTScheduler_h);
//	vTaskResume(vFw_Update_Mgr_Task_h);
}

void InitNormalOperations(void)
{
	// Check deployment state
	uint8_t deployment_state;
	getDeploymentStartupState(&deployment_state);
	if(deployment_state == DPL_STATE_STOWED)
	{
		InitiateSpacecraftDeployment();
		setDeploymentStartupState(DPL_STATE_DEPLOYED);
	}

	// Resume tasks
//	vTaskResume(vCanServer_h);
//	vTaskResume(vTTTScheduler_h);
//	vTaskResume(vFw_Update_Mgr_Task_h);
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
