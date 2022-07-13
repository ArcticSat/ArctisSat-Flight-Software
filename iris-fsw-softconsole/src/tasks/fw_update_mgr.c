//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// IrisSat 2019-2022
//
// Repository:
//  IrisSat-Flight-Software
//
// File Description:
//  Firmware Update Manager Module.
//
// History
// 2022-05  Joseph Howarth
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "tasks/fw_update_mgr.h"
#include "drivers/filesystem_driver.h"
#include "Libraries/libcrc/include/checksum.h"


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t fwMgrState = FW_STATE_IDLE;
static uint8_t fwMgrArmed = 0;
static uint8_t fwMgrExeConfirmed = 0;
static uint8_t rx_in_progress = 0;
static uint8_t rx_slot_index = 0;

static Fw_metadata_t fwFiles[4]; //We keep up to 4 fw. Golden + backup, Upgrade + backup.

static char * fwFileNames[4]={
        "goldenFW.spi",
        "updateFW.spi",
        "goldenFW_bak.spi",
        "updateFW_bak.spi"
};

fwVerificationData_t verifiedStatus;

QueueHandle_t fwDataQueue;
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
static int updateState(int state);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void checksum_file(uint32_t * out, char * filename){

    //Based on the libcrc example/test program.
    uint32_t crc_32_val = 0xffffffffL;
    char ch;
    unsigned char prev_byte;
    lfs_file_t file = {0};
    int result_fs = fs_file_open( &file, filename, LFS_O_RDWR);
    if(result_fs < 0){
        printf("Could not open file to checksum: %d\n",result_fs);
        return;
    }

    fs_file_seek(&file, 0, SEEK_SET);

    uint8_t byte_buff[256];
    //Use a buffer here to reduce filesystem overhead...
    //Size is arbitrary, doesn't seem to actually speed up for large files once above 1.

//    TickType_t start = xTaskGetTickCount();
    int count = 0;
    int bytes_to_process = 0;
    while( (bytes_to_process = fs_file_read(&file, byte_buff, 256)) >0 ) {

        for(int i =0; i < bytes_to_process; i++){
            crc_32_val = update_crc_32(     crc_32_val, byte_buff[i]);
            count ++;
        }

    }

    fs_file_seek(&file, 0, SEEK_SET);

    fs_file_close(&file);


    crc_32_val        ^= 0xffffffffL;

    *out = crc_32_val;

//    TickType_t time = xTaskGetTickCount()-start;
//    printf("Checksum took %d ms\n",time);
    printf("crc32'd %d bytes\n",count);
}


void checksum_program_flash_area(uint32_t address, uint32_t size){

    //Based on the libcrc example/test program.
    uint32_t crc_32_val = 0xffffffffL;
    char ch;
    unsigned char prev_byte;
    lfs_file_t file = {0};
    int result_fs = fs_file_open( &file, filename, LFS_O_RDWR);
    if(result_fs < 0){
        printf("Could not open file to checksum: %d\n",result_fs);
        return;
    }



    uint8_t byte_buff[256];
    //Use a buffer here to reduce filesystem overhead...
    //Size is arbitrary, doesn't seem to actually speed up for large files once above 1.

//    TickType_t start = xTaskGetTickCount();
    int count = 0;
    int bytes_left = size;
    int bytes_to_process = bytes_left>256 ? 256:bytes_left;
    int addr_curr = address;
    FlashStatus_t stat = flash_read(flash_devices[PROGRAM_FLASH],addr_curr, byte_buff, bytes_to_process);

    while( stat == FLASH_OK && bytes_to_process > 0) {

        for(int i =0; i < bytes_to_process; i++){
            crc_32_val = update_crc_32(     crc_32_val, byte_buff[i]);
            count ++;
        }
        bytes_left = bytes_left-bytes_to_process;
        bytes_to_process = bytes_left>256 ? 256:bytes_left;
        stat = flash_read(flash_devices[PROGRAM_FLASH],addr_curr, byte_buff, bytes_to_process);

    }

    crc_32_val        ^= 0xffffffffL;

    *out = crc_32_val;

//    TickType_t time = xTaskGetTickCount()-start;
//    printf("Checksum took %d ms\n",time);
    printf("crc32'd %d bytes\n",count);
}


void vFw_Update_Mgr_Task(void * pvParams){


    lfs_file_t fwfile = {0};
    int rx_byte_index=0;
    uint8_t rxDataBuff[FW_CHUNK_SIZE];
    char tempFileName[64];
    uint8_t fwTempFileOpen = 0;


    fwDataQueue = xQueueCreate(5,sizeof(uint8_t)*FW_CHUNK_SIZE);
    if(fwDataQueue == NULL){
       printf("Cannot create fw data queue! Fw updates will not work...\n");
    }

    //Check the persistent memory to bring back any previous state
    initializeFwMgr();

    //Run the state machine.
    while(1){

        switch(fwMgrState){

            case FW_STATE_IDLE:{
                //In idle mode, delay to allow other tasks to run.
                vTaskDelay(500);
                break;
            }
            case FW_STATE_RX_FW:{

                if(!rx_in_progress){
                   //Our metadata wasn't received properly, go back to idle.
                    updateState(FW_STATE_IDLE);
                    //Log error
                    printf("Exiting RX_FW state, no update metadata received.\n");
                }
                else{


                    //Open up the file.
                    if(!fwTempFileOpen){

                        snprintf(tempFileName,64,"%s.tmp",fwFileNames[rx_slot_index]);//We will upload our file to .tmp. once complete, delete original and rename the temp file.

                        int result_fs = fs_file_open( &fwfile, tempFileName, LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC);//For now truncate if already exists.
                        if(result_fs < 0){
                            printf("Could not open temp file to write fw to: %d\n",result_fs);
                            updateState(FW_STATE_IDLE);
                            break;
                        }
                        fwTempFileOpen=1;
                    }

                    //Now we can wait for data and write to the file.
                    BaseType_t res = xQueueReceive(fwDataQueue,rxDataBuff,pdMS_TO_TICKS(10000));

                    if(res == pdTRUE){

                        fs_file_write( &fwfile, &rxDataBuff, sizeof(rxDataBuff));
                        rx_byte_index += FW_CHUNK_SIZE;

                        if(rx_byte_index >= fwFiles[rx_slot_index].filesize){

                            fs_file_sync(&fwfile);

                            //If file is not multiple of chunk size then we will have extra data(garbage) at the end, so truncate.
                            if(rx_byte_index > fwFiles[rx_slot_index].filesize){
                                fs_file_truncate(&fwfile, fwFiles[rx_slot_index].filesize);
                            }

                            fs_file_close( &fwfile);



                            //Now calculate the checksum;
                            uint32_t check = 0;
                            checksum_file(&check,tempFileName);

                            if(check == fwFiles[rx_slot_index].checksum){

                                //now delete the actual fw, and rename the temp file.
                                //Check file exists first...

                                if(fs_file_exist(fwFileNames[rx_slot_index])){
                                    int res = fs_remove(fwFileNames[rx_slot_index]);
                                    if(res<0){
                                        printf("FwMgr: Could not finish uploading fw, cant remove original: %d \n",res);
                                        updateState(FW_STATE_IDLE);
                                        break;
                                    }
                                }
                                res = fs_rename(tempFileName, fwFileNames[rx_slot_index]);
                                if(res<0){
                                    printf("FwMgr: Could not finish uploading fw, cant rename temp: %d\n",res);
                                    updateState(FW_STATE_IDLE);
                                    break;
                                }


//                                if(fs_file_exist(fwFileNames[rx_slot_index])){
//                                    res = fs_remove(tempFileName);
//                                    if(res<0){
//                                        printf("FwMgr: Could not finish uploading fw, cant remove temp: %d\n",res);
//                                        updateState(FW_STATE_IDLE);
//                                        break;
//                                    }
//                                }
                                //If we make it here we are done!
                                rx_byte_index  =0;
                                rx_in_progress =0;
                                fwTempFileOpen =0;
                                updateState(FW_STATE_IDLE);
                            }
                        }

                    }

                }



                break;
            }
            case FW_STATE_PRE_VERIFY:{


                for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

                    verifiedStatus.verified[i] = 0;

                    uint32_t check = 0;
                    checksum_file(&check,fwFileNames[i]);
                    if(check == fwFiles[i].checksum && fwFiles[i].filesize>0 ){
                        verifiedStatus.verified[i]=1;
                    }
                    printf("Firmware %d (%s) verify: %s",i,fwFileNames[i],(verifiedStatus.verified[i]?"PASS":"FAIL"));

                }

                //Now based on the status we can attempt to fix our problem or alert ground.

                //First we see if we can copy backup to replace original:
                for(int i=0; i<NUM_FIRMWARES; i++){
                    //Backup images are kept sequentially after originals(golden->0, upgrade->1, golden backup->2 etc.)
                    if(verifiedStatus.verified[i] ==0 && verifiedStatus.verified[i+2]==1){
                        //We have a corrupted original so just copy the backup to replace the orig.
                        fs_copy_file(fwFileNames[i+2], fwFileNames[i]);
                        fwFiles[i+2].checksum = fwFiles[i].checksum;
                        fwFiles[i+2].filesize = fs_file_size_from_path(fwFileNames[i+2]);
                    }
                }

                //Now check the opposite, if our backup is missing/bad, then replace with the orig:
                for(int i=0; i<NUM_FIRMWARES; i++){
                    //Backup images are kept sequentially after originals(golden->0, upgrade->1, golden backup->2 etc.)
                    if(verifiedStatus.verified[i+2] ==0 && verifiedStatus.verified[i]==1){
                        //We have a corrupted original so just copy the backup to replace the orig.
                        fs_copy_file(fwFileNames[i], fwFileNames[i+2]);
                        fwFiles[i].checksum = fwFiles[i+2].checksum;
                        fwFiles[i].filesize = fs_file_size_from_path(fwFileNames[i]);
                    }
                }

                //At this point we should have all images verified, if not then we cannot fix without ground uploading a new file(s).
                for(int i=0; i< NUM_FIRMWARES;i++){

                    if(verifiedStatus.verified[i] != 1){
                        printf("Unrecoverable error: %s is missing or corrupt and cannot be recovered. Upload a new file...\n",fwFileNames[i]);
                        updateState(FW_STATE_IDLE);
                        break;
                        //log error.
                    }
                }



                //Lastly we must sync up the program flash files, so repeat similar steps here.


                //Run checksum on program flash, we need a slightly different checksum function to do this since we have no fs on the program flash.

                //Then if program firmware fails checksum and we have a good copy, then copy it over... again we need a modified copy_file function.
                updateState(FW_STATE_IDLE);
                break;
            }
            case FW_STATE_ARMED:{

                break;
            }
            case FW_STATE_UPDATE:{

                break;
            }
            case FW_STATE_POST_VERIFY:{

                break;
            }

        }


    }
}

void listFwFiles(){

    for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

        printf("Slot %d: %s (%d b) (%0x)\n",fwFiles[i].fileIndex,fwFileNames[i],fwFiles[i].filesize,fwFiles[i].checksum);
    }

}

int updateFwMetaData(Fw_metadata_t* data){

    int res = -1;
    if(data->fileIndex <= 1 && data->fileIndex >= 0){
        memcpy(&fwFiles[data->fileIndex],data, sizeof(Fw_metadata_t));
        res = 0;
        rx_in_progress = 1;
        rx_slot_index = data->fileIndex;

        lfs_file_t checkFile = {0};
        char checksumFileName[64]={0};

        sprintf(checksumFileName,"%s.check",fwFileNames[rx_slot_index]);
        int result_fs = fs_file_open(&checkFile,checksumFileName, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
        if(result_fs <0){
            printf("Could not open file to store checksum\n ");
            return -1;
        }
        fs_file_write(&checkFile, &data->checksum, sizeof(uint32_t));
        fs_file_close(&checkFile);

    }

        return res;
}

int getFwManagerState(){

    return fwMgrState;
}

int setFwManagerState(int state){

    int res = updateState(state);
    if(getFwManagerState() != state || res<0){
        printf("Unable to set state!");
    }
}

int updateState(int state){

    int setState = state;

    switch(fwMgrState){

        case FW_STATE_IDLE:{

            //From IDLE we are not allowed to go to UPDATE, otherwise any state is valid.
            if(state == FW_STATE_UPDATE) return -1;

            //Special case: If we go to ARMED, we should pre-verify first, and then set armed.
            if(state == FW_STATE_ARMED){
                setState = FW_STATE_PRE_VERIFY;
                fwMgrArmed = 1;
            }

            break;
        }
        case FW_STATE_RX_FW:{

            //Only valid choice is back to idle.
            if(state != FW_STATE_IDLE) return -1;

            break;
        }
        case FW_STATE_PRE_VERIFY:{

            if(state != FW_STATE_IDLE && state != FW_STATE_ARMED) return -1;

            if(state == FW_STATE_ARMED){
                fwMgrArmed = 1;
            }

            break;
        }
        case FW_STATE_ARMED:{

            if(state != FW_STATE_IDLE && state != FW_STATE_UPDATE) return -1;
            break;
        }
        case FW_STATE_UPDATE:{

            if(state != FW_STATE_IDLE) return -1;
            break;
        }
        case FW_STATE_POST_VERIFY:{

            if(state != FW_STATE_IDLE) return -1;
            break;
        }

    }

    fwMgrState = state;


}

void uploadFwChunk(uint8_t * data, uint16_t length){

    if(length>FW_CHUNK_SIZE){
        printf("fw chunk larger than chunk size!Not ok\n");
        return;
    }
    xQueueSendToBack(fwDataQueue,data,100);
}

void initializeFwMgr(){

    //Load/calculate our file metadata
    for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

        lfs_file_t checkFile = {0};
        char checksumFileName[64]={0};

        sprintf(checksumFileName,"%s.check",fwFileNames[i%2]);
        int result_fs = fs_file_open(&checkFile,checksumFileName, LFS_O_RDONLY);
        if(result_fs < 0 && result_fs  == LFS_ERR_NOENT){
            printf("Could not open checksum file %s to load: %d\n Creating File...",checksumFileName,result_fs);

            fs_file_open(&checkFile, checksumFileName, LFS_O_CREAT);

        }else if (result_fs<0){

           printf("Could not open checksum file %s to load: %d\n",checksumFileName,result_fs);
           continue;
        }
        uint32_t check =0;
        if (fs_file_read(&checkFile,&check,sizeof(uint32_t)) != sizeof(uint32_t)){
            printf("Checksum incorrect size\n");
            check = 0;
            //What to do here? I guess leave checksum as 0, which should be impossibly rare to be valid.
        }

        fwFiles[i].checksum = check;
        fs_file_close(&checkFile);

    }

    for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

//        lfs_file_t file = {0};
//        struct lfs_info info ={0};
//
//        fs_stat(fwFileNames[i], &info);

        fwFiles[i].filesize = fs_file_size_from_path(fwFileNames[i]);
        fwFiles[i].fileIndex = i;
        printf("fs_stat: %s %d",fwFileNames[i],fwFiles[i].filesize);
    }

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
