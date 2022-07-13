#ifndef FW_UPDATE_MGR_H
#define FW_UPDATE_MGR_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Firmware Update Manager.
//
// History
// 2022-05 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "FreeRTOS.h"
#include "queue.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//For testing only. Comment this out!
//With this we don't keep backup of the firmware, to reduce flash needed.
#define NO_FW_BACKUP

#define FW_CHUNK_SIZE   150
#define NUM_FIRMWARES       2 //We keep a golden image and an update image.
#define FW_ARMED_TIMEOUT_MS  (10 * 60*1000) //10 minutes

#ifdef NO_FW_BACKUP
#define NUM_FIRMWARES_TOTAL (NUM_FIRMWARES)  //Each firmware does NOT have backup
#else
#define NUM_FIRMWARES_TOTAL (NUM_FIRMWARES*2)  //Each firmware has a backup
#endif
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

typedef enum{

    FW_STATE_IDLE,
    FW_STATE_RX_FW,
    FW_STATE_PRE_VERIFY,
    FW_STATE_ARMED,
    FW_STATE_UPDATE,
    FW_STATE_POST_VERIFY,

} fwMgrState_t;


extern QueueHandle_t fwDataQueue;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

typedef struct{
    uint8_t fileIndex; //Valid options are 0(GOLDEN image) or 1 (UPDATE image)
    uint32_t filesize;//Filesize in bytes. For use in receiving and as extra check.
    uint32_t checksum;//crc-32 full file checksum.

}Fw_metadata_t;

typedef struct {
    uint8_t verified[4];
}fwVerificationData_t;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Description: This task manages the firmware update process
//
//  Stack Size: The task should be initialized with n word stack size.
//  Parameters:
//------------------------------------------------------------------------------
void vFw_Update_Mgr_Task(void * pvParams);

void listFwFiles();
int updateFwMetaData(Fw_metadata_t* data);

int getFwManagerState();
int setFwManagerState(int state);
void uploadFwChunk(uint8_t * data, uint16_t length);//Will length ever be larger than 256? should be smaller than chunk size...
void checksum_file(uint32_t * out, char * filename);


#endif // FW_UPDATE_MGR_H
