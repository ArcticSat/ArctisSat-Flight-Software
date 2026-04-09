/*
 * telemetry_manager.c
 *
 *  Created on: Dec. 2, 2022
 *      Author: jpmckoy
 */

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "application/memory_manager.h"
#include "drivers/filesystem_driver.h"
#include "application/sc_deployment.h"
#include "drivers/device/memory/flash_common.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define MM_FILENAME_MAX 50
#define NUM_FILES 2

//We will keep the sc status in the last sector of memory.
#define SC_STATUS_LOCATION  (flash_devices[PROGRAM_FLASH]->device_size-flash_devices[PROGRAM_FLASH]->erase_size)
#define MEM_MANAGER_FLASH_DEVICE    (flash_devices[PROGRAM_FLASH])
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	SC_STATUS_FILE_IDX,
	CDH_TM_LOG_FILE_IDX
} eFilenameIndex;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static bool verbosity = true;
const char * filenames[] =
{
	"spacecraft_status",
	"cdh_tm_log"
};

ScStatus_t scStatus = {0};
int sc_status_read_result_fs=-1;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static int readScStatus(ScStatus_t* sc);
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

void init_memory_manager(void)
{

}

void set_telemetry_verbose(bool verbose)
{
	verbosity = verbose;
}
void log_telemetry(telemetryPacket_t * pkt)
{

}

void get_telemetry(TelemetryChannel_t channel_id)
{

}

void log_event(telemetryPacket_t * pkt)
{

}

/*** Spacecraft status utilities ***/
int InitSpacecraftStatus(void)
{
    sc_status_read_result_fs = readScStatus(&scStatus);


    return  sc_status_read_result_fs; // If state is zero, then detumble
}

int getScStatus(ScStatus_t * sc_status)
{
	memcpy(sc_status,&scStatus,sizeof(ScStatus_t));
	return sc_status_read_result_fs;
}

int readScStatus(ScStatus_t* sc){

    int res = flash_read(MEM_MANAGER_FLASH_DEVICE,SC_STATUS_LOCATION,(uint8_t*)sc,sizeof(ScStatus_t));

    //Also we should check the data, since the flash read could return but we could have no actual data.
    if(scStatus.deployment_state == 0xFF && scStatus.detumble_state ==0xFF && scStatus.last_reboot_reason ==0xFF){
        //Likely flash is not working or we lost data somehow or it is first time.
        res = MEM_MGR_READ_ERR;
    }

    return  res;
}


int CommitSpacecraftStatus(void)
{
	flash_erase(MEM_MANAGER_FLASH_DEVICE,SC_STATUS_LOCATION);
	sc_status_read_result_fs = flash_write(MEM_MANAGER_FLASH_DEVICE, SC_STATUS_LOCATION, (uint8_t*)&scStatus, sizeof(ScStatus_t));

	ScStatus_t readBack={0xFF};
	sc_status_read_result_fs = readScStatus(&readBack);

	if(scStatus.deployment_state != readBack.deployment_state || scStatus.detumble_state != readBack.detumble_state || scStatus.last_reboot_reason != readBack.last_reboot_reason){
	    sc_status_read_result_fs = MEM_MGR_WRITE_ERR;
	}

	return sc_status_read_result_fs; // If state is zero, then detumble
}


/*** Deployment status ***/
int getDeploymentStartupState(uint8_t * state)
{
	if(sc_status_read_result_fs != FS_OK)
	{
		*state = DPL_STATE_DEPLOYED;
	}
	else
	{
		*state = scStatus.deployment_state;
	}

	return sc_status_read_result_fs;
}
int setDeploymentStartupState(uint8_t state)
{
	scStatus.deployment_state = state;
	return CommitSpacecraftStatus();
}

/*** Detumbling status ***/
int getDetumblingStartupState(uint8_t * state)
{
	if(sc_status_read_result_fs != FS_OK)
	{
		*state = DETUMBLING_COMPLETE;
	}
	else
	{
		*state = scStatus.detumble_state;
	}

	return sc_status_read_result_fs;
}
int setDetumblingStartupState(uint8_t state)
{
	scStatus.detumble_state = state;
	return CommitSpacecraftStatus();
}
//Reboot Reason
int getLastRebootReason(uint8_t *reason){

    if(sc_status_read_result_fs != FS_OK)
    {
        *reason = REBOOT_UNKNOWN;
    }
    else
    {
        *reason = scStatus.last_reboot_reason;
    }

    return sc_status_read_result_fs;

}
int setLastRebootReason(uint8_t  reason){

    scStatus.last_reboot_reason = reason;
    return CommitSpacecraftStatus();
}

