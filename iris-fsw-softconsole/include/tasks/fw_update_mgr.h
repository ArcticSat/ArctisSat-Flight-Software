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
//#define NO_FW_BACKUP

#define FW_CHUNK_SIZE   150
#define NUM_FIRMWARES       2 //We keep a golden image and an update image.
#define FW_ARMED_TIMEOUT_MS  (2 * 60*1000) //10 minutes

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

typedef enum{

    GODLEN_LOCATION,  
    UPDATE_LOCATION_LOCATION,
    GOLDEN_BACKUP_LOCATION,
    UPDATE_BACKUP_LOCATION,

}fwMgrDataLocation;


extern QueueHandle_t fwDataQueue;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

typedef struct{
    uint8_t fileIndex; //Valid options are 0(GOLDEN image) or 1 (UPDATE image)
    uint32_t filesize;//Filesize in bytes. For use in receiving and as extra check.
    uint32_t checksum;//crc-32 full file checksum.
    uint8_t designver;//FPGA design version.

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

// Will update the data(checksum,size, etc. about the two firmware images.
int updateFwMetaData(Fw_metadata_t* data);

int getFwManagerState();
int setFwManagerState(int state);

//Saves a chunk of data to one of the firmware files
void uploadFwChunk(uint8_t * data, uint16_t length);//Will length ever be larger than 256? should be smaller than chunk size...

//Computes the checksum of a file.
void checksum_file(uint32_t * out, char * filename);

//Will run the checksum algorithm on the program flash, starting at the specified address and using "size" bytes.
void checksum_program_flash_area(uint32_t *out,uint32_t address, uint32_t size);

//This will copy a file to the program flash at the specified address.
int  copy_to_prog_flash(char * filename, uint32_t address);

//This will actually do the upgrade. Must be in the UPDATE stat, then call this function to confirm the upgrade.
void execute_confirm();

//This will update which image we will upgrade to. 0-> Golden, 1-> Update.
void setFwTargetImage(uint8_t target);

//Will reset the firmware update manager and try to recover any previous info.
void initializeFwMgr();

//In case we need to manually assign the checksum for the firmware.
//Slot  is 0(golden), 1(update).
void setFwChecksum(uint8_t slot, uint32_t check);

//In case we need to manually assign the design version for the firmware.
//Slot  is 0(golden), 1(update).
void setFwDesignVer(uint8_t slot, uint8_t ver);


#endif // FW_UPDATE_MGR_H
