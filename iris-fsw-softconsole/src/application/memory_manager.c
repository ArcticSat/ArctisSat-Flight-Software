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

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define MM_FILENAME_MAX 50
#define NUM_FILES 2
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct  {
	uint8_t deployment_state;
	uint8_t detumble_state;
} ScStatus_t;
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
int sc_status_read_result_fs;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

void init_memory_manager(void)
{
	// Verbosity
#ifdef TM_VERBOSITY
	verbosity = true;
#else
	verbosity = false;
#endif
	// File creation
	int i;
    int result_fs;
	lfs_file_t file = {0};
	for(i=0; i < NUM_FILES; i++){
		result_fs = fs_file_open(&file, filenames[i], LFS_O_CREAT);
		result_fs = fs_file_close(&file);
	}
}

void set_telemetry_verbose(bool verbose)
{
	verbosity = verbose;
}
void log_telemetry(telemetryPacket_t * pkt)
{
	// Debugging
	if (verbosity)
		sendTelemetryAddr(pkt, GROUND_CSP_ADDRESS);

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
	lfs_file_t file = {0};
	// Open detumble_state file
	char filename[MM_FILENAME_MAX];
	sprintf(filename,"%s",filenames[SC_STATUS_FILE_IDX]);
//	const char * filename = filenames[SC_STATUS_FILE_IDX];
	sc_status_read_result_fs = fs_file_open( &file, filename, LFS_O_RDWR);
	if(sc_status_read_result_fs != FS_OK) return sc_status_read_result_fs;
	// Read state
	sc_status_read_result_fs = fs_file_read( &file, (void *) &scStatus, sizeof(scStatus));
	if(sc_status_read_result_fs != FS_OK) return sc_status_read_result_fs;
	// Close file
	sc_status_read_result_fs = fs_file_close( &file);
	// Return
	return sc_status_read_result_fs; // If state is zero, then detumble
}
int CommitSpacecraftStatus(void)
{
	if(sc_status_read_result_fs != FS_OK)
	{
		return sc_status_read_result_fs;
	}
	lfs_file_t file = {0};
	// Open detumble_state file
	char filename[MM_FILENAME_MAX];
	sprintf(filename,"%s",filenames[SC_STATUS_FILE_IDX]);
//	const char * filename = filenames[SC_STATUS_FILE_IDX];
	sc_status_read_result_fs = fs_file_open( &file, filename, LFS_O_RDWR);
	if(sc_status_read_result_fs != FS_OK) return sc_status_read_result_fs;
	// Read state
	sc_status_read_result_fs = fs_file_write( &file, (void *) &scStatus, sizeof(scStatus));
	if(sc_status_read_result_fs != FS_OK) return sc_status_read_result_fs;
	// Close file
	sc_status_read_result_fs = fs_file_close( &file);
	// Return
	return sc_status_read_result_fs; // If state is zero, then detumble
}


/*** Deployment status ***/
int getDeploymentStartupState(uint8_t * state)
{
	if(sc_status_read_result_fs != FS_OK)
	{
		*state = DPL_STATE_STOWED;
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


