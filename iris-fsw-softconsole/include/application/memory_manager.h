/*
 * telemetry_manager.h
 *
 *  Created on: Dec. 2, 2022
 *      Author: jpmckoy
 */

#ifndef INCLUDE_APPLICATION_MEMORY_MANAGER_H_
#define INCLUDE_APPLICATION_MEMORY_MANAGER_H_


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "tasks/telemetry.h"
#include "application/detumbling.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TM_VERBOSITY
#define EVENT_DATA_SIZE 4

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SC_STATUS_SIZE_BYTES 3
typedef struct  {
	uint8_t deployment_state;
	uint8_t detumble_state;
	uint8_t last_reboot_reason;
} ScStatus_t;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Telemetry channels
typedef enum {
	SC_STATUS,
	EVENT_LOG,
	CDH_CHANNEL,
	POWER_TLM_CHANNEL,
	PAYLOAD_TLM_CHANNEL,
	ADCS_TLM_CHANNEL,
	NUM_TLM_CHANNELS,
} TelemetryChannel_t;

// Event IDs
typedef enum
{
	TASK_ERROR,
	TASK_SUCCESS
} EventId_t;

typedef enum
{
    REBOOT_UNKNOWN=0x1, //Default reason
    REBOOT_INTERNAL_WD=0x2,
    REBOOT_STACK_OVERFLOW=0x4,
    REBOOT_USER_SOFT_RESET=0x8,
    REBOOT_OTA_UPDATE=0x10,
    REBOOT_POWER_REQUEST=0x20,

} RebootReason_t;

typedef enum{
    MEM_MGR_OK=0,
    MEM_MGR_NOT_INIT=-1,
    MEM_MGR_READ_ERR=-2,
    MEM_MGR_WRITE_ERR=-3,

}MemManagerError_t;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBALS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void init_memory_manager(void);
void set_telemetry_verbose(bool verbose);
void log_telemetry(telemetryPacket_t * pkt);
void get_telemetry(TelemetryChannel_t channel_id);
void log_event(telemetryPacket_t * pkt);
/*** Spacecraft status utilities ***/
int InitSpacecraftStatus(void);
int getScStatus(ScStatus_t * sc_status);
int CommitSpacecraftStatus(void);
// Deployment status
int getDeploymentStartupState(uint8_t * state);
int setDeploymentStartupState(uint8_t state);
// Detumbling status
int getDetumblingStartupState(uint8_t * state);
int setDetumblingStartupState(uint8_t state);
//Reboot Reason
int getLastRebootReason(uint8_t *reason);
int setLastRebootReason(uint8_t  reason);

#endif /* INCLUDE_APPLICATION_MEMORY_MANAGER_H_ */
