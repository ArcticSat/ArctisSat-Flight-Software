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
#include "drivers/device/memory/flash_common.h"
#include "Libraries/libcrc/include/checksum.h"
#include "drivers/software_update_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks/telemetry.h"
#include "application/memory_manager.h"
#include "drivers/device/watchdog.h"


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t fwMgrState = FW_STATE_IDLE;
static uint8_t fwMgrArmed = 0;
static int32_t fw_armed_timeout=0;
static uint8_t fwMgrExeConfirmed = 0;
static uint8_t rx_in_progress = 0;
static uint8_t rx_slot_index = 0;
static int rx_byte_index=0;
static uint8_t targetFw = 1;//Which image will we upgrade to. 1 is default for update image. 0 for golden.
static int UploadMode = FW_UPLOAD_REV2;
uint16_t curr_seq_num = 0;
static int32_t fw_user_timeout=0;
static uint8_t fw_update_attempts=0;

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
static int checksumAllFw(); //Runs checksum on the golden and update files, and their backups. The verifiedStatus global will be updated.
static void fwRxSendAck(int ackNak, uint16_t seq);



//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int checksumAllFw(){
    int result = 1;

    for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

        verifiedStatus.verified[i] = 0;

        uint32_t check = 0;
        checksum_file(&check,fwFileNames[i]);
        if(check == fwFiles[i].checksum && fwFiles[i].filesize>0 ){
            verifiedStatus.verified[i]=1;
        }
        printf("Firmware %d (%s) verify: %s  (%x, %x)",i,fwFileNames[i],(verifiedStatus.verified[i]?"PASS":"FAIL"),fwFiles[i].checksum,check);

        result &= verifiedStatus.verified[i];
    }
    return result;
}

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

    uint8_t byte_buff[512];
    //Use a buffer here to reduce filesystem overhead...
    //Size is arbitrary, doesn't seem to actually speed up for large files once above 1.

    TickType_t start = xTaskGetTickCount();
    int count = 0;
    int bytes_to_process = 0;
    while( (bytes_to_process = fs_file_read(&file, byte_buff, 512)) >0 ) {

        for(int i =0; i < bytes_to_process; i++){
            crc_32_val = update_crc_32(     crc_32_val, byte_buff[i]);
            count ++;
        }

    }

    fs_file_seek(&file, 0, SEEK_SET);

    fs_file_close(&file);


    crc_32_val        ^= 0xffffffffL;

    *out = crc_32_val;

    TickType_t time = xTaskGetTickCount()-start;
    printf("Checksum took %d ms\n",time);
    printf("crc32'd %d bytes\n",count);
}

void checksum_file_area(uint32_t* out, char * filename, uint32_t start_byte, uint32_t len, int quiet){

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

    fs_file_seek(&file, start_byte, SEEK_SET);

    uint8_t byte_buff[256];
    //Use a buffer here to reduce filesystem overhead...
    //Size is arbitrary, doesn't seem to actually speed up for large files once above 1.

    TickType_t start = xTaskGetTickCount();
//    int count = 0;
//    int bytes_to_process = 0;
//    while( (bytes_to_process = fs_file_read(&file, byte_buff, 512)) >0 ) {
//
//        for(int i =0; i < bytes_to_process; i++){
//            crc_32_val = update_crc_32(     crc_32_val, byte_buff[i]);
//            count ++;
//        }
//
//    }

      int count = 0;
      int bytes_left = len;
      int bytes_to_process = bytes_left>256 ? 256:bytes_left;
      int addr_curr = start_byte;
      int actual = fs_file_read(&file, byte_buff, bytes_to_process);

      while( actual>0 && bytes_to_process > 0) {

          for(int i =0; i < actual; i++){
              crc_32_val = update_crc_32(     crc_32_val, byte_buff[i]);
              count ++;
          }
          addr_curr += bytes_to_process;
          bytes_left = bytes_left-bytes_to_process;
          bytes_to_process = bytes_left>256 ? 256:bytes_left;

          actual = fs_file_read(&file, byte_buff, bytes_to_process);

      }

    fs_file_seek(&file, 0, SEEK_SET);

    fs_file_close(&file);


    crc_32_val        ^= 0xffffffffL;

    *out = crc_32_val;

    TickType_t time = xTaskGetTickCount()-start;
    if(!quiet){
        printf("Checksum took %d ms\n",time);
        printf("crc32'd %d bytes\n",count);
    }
}

void checksum_program_flash_area(uint32_t *out,uint32_t address, uint32_t size){

    //Based on the libcrc example/test program.
    uint32_t crc_32_val = 0xffffffffL;
    char ch;
    unsigned char prev_byte;

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
        addr_curr += bytes_to_process;
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



int copy_to_prog_flash(char * filename, uint32_t address){

    uint32_t filesize = fs_file_size_from_path(filename);
    uint32_t chunksize = 256;
    uint8_t byte_buff[256] = {0};
    int addr_curr = address;

    lfs_file_t file = {0};
    int result_fs = fs_file_open( &file, filename, LFS_O_RDONLY);
    if(result_fs < 0){
        printf("%s: Could not open fw file to read: %d\n",__FUNCTION__,result_fs);
        return -1;
    }


    //Erase the area first.
    for(int i=0; i< filesize/4096 + (filesize % 4096 > 0);i++){
        flash_erase(flash_devices[PROGRAM_FLASH],address+(i*4096));
    }

    flash_read(flash_devices[PROGRAM_FLASH],4096,byte_buff,256);

    int bytes_to_process = 0;
    while( (bytes_to_process = fs_file_read(&file, byte_buff, chunksize)) >0 ) {
       flash_write(flash_devices[PROGRAM_FLASH],addr_curr,byte_buff,bytes_to_process);
       addr_curr += bytes_to_process;
    }

    fs_file_close(&file);
    return 0;

}


void vFw_Update_Mgr_Task(void * pvParams){


    lfs_file_t fwfile = {0};

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
                    if(!fs_is_open(&fwfile)){

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

                        if(UploadMode == FW_UPLOAD_REV2){

                            //First thing is to unpack the data and extra info.
                            uint16_t seq_num =0;
                            memcpy(&seq_num,&rxDataBuff[0],sizeof(uint16_t));
                            uint8_t* data = &rxDataBuff[sizeof(uint16_t)];
                            uint8_t dataSize = sizeof(rxDataBuff)-sizeof(uint16_t);
                            int seek =0;
                            int ack_nak = 1;

                            if(seq_num !=curr_seq_num){
                                //We have missed a packet.
                                printf("FwMgr: missed packet or ooo | rx %d expect %d\n",seq_num,curr_seq_num);
                                ack_nak = 0;
                            }
                            if(ack_nak){
                                int write_res =fs_file_write( &fwfile, data, dataSize );
                                if(write_res <0){
                                    printf("FwMgr: Problem writing fw chunk to file: %d \n",write_res);
                                    updateState(FW_STATE_IDLE);
                                    break;
                                }
                                rx_byte_index += dataSize;
                            }

                            fwRxSendAck(ack_nak,curr_seq_num);
                            if(ack_nak)curr_seq_num++;


                        }
                        else{

                            int write_res =fs_file_write( &fwfile, &rxDataBuff, sizeof(rxDataBuff));
                            if(write_res <0){
                                printf("FwMgr: Problem writing fw chunk to file: %d \n",write_res);
                                updateState(FW_STATE_IDLE);
                                break;
                            }
                            rx_byte_index += FW_CHUNK_SIZE;
                        }
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
                                curr_seq_num=0;
                                updateState(FW_STATE_IDLE);
                            }
                        }

                    }

                }

                if(fwMgrState == FW_STATE_IDLE){
                    //If we're done in RX state, make sure we close the temp file. Catches case where we resetFwMgr, but file doesn't get closed.
                    if(fs_is_open(&fwfile)) fs_file_close(&fwfile);
                }

                break;
            }
            case FW_STATE_PRE_VERIFY:{

                int exit = 0;

                //Run through the files and do the checksums.
                checksumAllFw();

                //Now based on the status we can attempt to fix our problem or alert ground.
#ifndef NO_FW_BACKUP
                //First we see if we can copy backup to replace original:
                for(int i=0; i<NUM_FIRMWARES; i++){
                    //Backup images are kept sequentially after originals(golden->0, upgrade->1, golden backup->2 etc.)
                    if(verifiedStatus.verified[i] ==0 && verifiedStatus.verified[i+2]==1){
                        //We have a corrupted original so just copy the backup to replace the orig.
                        fs_copy_file(fwFileNames[i+2], fwFileNames[i]);
                        fwFiles[i].checksum = fwFiles[i+2].checksum;
                        fwFiles[i].filesize = fs_file_size_from_path(fwFileNames[i+2]);
                    }
                }

                //Now check the opposite, if our backup is missing/bad, then replace with the orig:
                for(int i=0; i<NUM_FIRMWARES; i++){
                    //Backup images are kept sequentially after originals(golden->0, upgrade->1, golden backup->2 etc.)
                    if(verifiedStatus.verified[i+2] ==0 && verifiedStatus.verified[i]==1){
                        //We have a corrupted original so just copy the backup to replace the orig.
                        fs_copy_file(fwFileNames[i], fwFileNames[i+2]);
                        fwFiles[i+2].checksum = fwFiles[i].checksum;
                        fwFiles[i+2].filesize = fs_file_size_from_path(fwFileNames[i]);
                    }
                }
#endif

                checksumAllFw();

                //At this point we should have all images verified, if not then we cannot fix without ground uploading a new file(s).
                for(int i=0; i< NUM_FIRMWARES_TOTAL;i++){

                    if(verifiedStatus.verified[i] != 1){
                        printf("Unrecoverable error: %s is missing or corrupt and cannot be recovered. Upload a new file...\n",fwFileNames[i]);
                        updateState(FW_STATE_IDLE);
                        exit = 1;
                        //log error.
                    }
                }

                if(exit) break;


                //Lastly we must sync up the program flash files, so repeat similar steps here.
                uint32_t check=0;
                checksum_program_flash_area(&check,FIRMWARE_GOLDEN_ADDRESS, fwFiles[0].filesize);

                if(check != fwFiles[0].checksum){
                    //Our program flash copy is bad, copy from the verified data flash, which at this point is verified.
                    if(copy_to_prog_flash(fwFileNames[0],FIRMWARE_GOLDEN_ADDRESS)){
                        printf("Unrecoverable error: problem copying fw to program flash\n");
                        updateState(FW_STATE_IDLE);
                        break;
                    }

                    check = 0;
                    checksum_program_flash_area(&check,FIRMWARE_GOLDEN_ADDRESS, fwFiles[0].filesize);

                    if(check != fwFiles[0].checksum){
                        //Something is wrong here return to idle state :(
                        printf("Unrecoverable error: bad checksum on program flash golden image, even after copying verified version\n");
                        updateState(FW_STATE_IDLE);
                        break;
                    }
                }

                checksum_program_flash_area(&check,FIRMWARE_UPDATE_ADDRESS, fwFiles[1].filesize);
                if(check != fwFiles[1].checksum){
                    //Our program flash copy is bad, copy from the verified data flash, which at this point is verified.
                    if (copy_to_prog_flash(fwFileNames[1],FIRMWARE_UPDATE_ADDRESS)<0){
                        printf("Unrecoverable error: problem copying fw to program flash\n");
                        updateState(FW_STATE_IDLE);
                        break;
                    }


                    check = 0;
                    checksum_program_flash_area(&check,FIRMWARE_UPDATE_ADDRESS, fwFiles[1].filesize);

                    if(check != fwFiles[1].checksum){
                        //Something is wrong here return to idle state :(
                        printf("Unrecoverable error: bad checksum on program flash update image, even after copying verified version\n");
                        updateState(FW_STATE_IDLE);
                        break;
                    }
                }

                update_spi_dir(0, fwFiles[0].designver);
                update_spi_dir(1, fwFiles[1].designver);

               //If we make it here we should have verified all copies of the firmware, so we can say system is armed!
               if(fwMgrArmed){
                   updateState(FW_STATE_ARMED);
                   fw_armed_timeout = (fw_user_timeout>0)?fw_user_timeout:FW_ARMED_TIMEOUT_MS;
               }
               else{
                   updateState(FW_STATE_IDLE);
                   break;
               }

                break;
            }
            case FW_STATE_ARMED:{

                //Here we just check for timeout. The longer we stay armed, the more chance of corruption. But if we timeout to soon then it becomes very slow to do the upgrade.

                if(fw_armed_timeout <0){
                    for(int i=0; i<NUM_FIRMWARES_TOTAL;i++){
                        verifiedStatus.verified[i]=0;
                    }
                    updateState(FW_STATE_IDLE);
                }

                TickType_t now = xTaskGetTickCount();
                vTaskDelayUntil(&now, 500);
                fw_armed_timeout -= 500;

                break;
            }
            case FW_STATE_UPDATE:{

                if(fwMgrExeConfirmed){

                    //First check our file is verified. We shouldn't be allowed here without the fw being verified, but just in case...
//                    if(verifiedStatus.verified[1] != 1){
//                        printf("Unverified FW in UPDATE state. This is not allowed, and shouldn't be  possible.\n");
//                        updateState(FW_STATE_IDLE);
//                        break;
//                    }

                    update_spi_dir(targetFw, fwFiles[targetFw].designver);

                    //Update state so we know to post verify on reboot.
                    int res = setLastRebootReason(REBOOT_OTA_UPDATE);
                    if(fw_update_attempts<1 && res != MEM_MGR_OK ){

                        printf("Could not set reboot reason, abort update. Try again to force, you must manually powercycle cdh!\n");
                        fw_update_attempts ++;
                        updateState(FW_STATE_IDLE);
                    }

                    fw_update_attempts=0; //We will reboot so this should get cleared anyways.

                    //Do one last check of the fw before we upload
                    uint32_t check = 0;
                    checksum_program_flash_area(&check,targetFw ? FIRMWARE_UPDATE_ADDRESS : FIRMWARE_GOLDEN_ADDRESS, fwFiles[targetFw].filesize);
                    if(check != fwFiles[targetFw].checksum){
                        printf("Unrecoverable error: bad checksum on program flash update image. How did this happen?\n");
                        updateState(FW_STATE_IDLE);
                        break;
                    }


                    //Shutdown the system, whatever that means.
                    //We should save any time-tagged tasks, adcs state, anything else important that is in RAM.


                    //ShutdownSystem();
                    fs_unmount();

                    //Stop the external WD.
                    start_stop_external_wd(0);
                    vTaskDelay(1000);//Since WD is seperate task, give some time to make sure it runs.

                    initiate_firmware_update(targetFw);

                    //We should not ever get here...
                    printf("FW upgrade failed!\n");
                    updateState(FW_STATE_IDLE);
                }
                vTaskDelay(500);
                break;
            }
            case FW_STATE_POST_VERIFY:{

                uint16_t design_ver;
                //Check that our current firmware matches what we expect.
                int res = authenticate_firmware(1,&design_ver);
                if(res == 0){
                    printf("FW update post verified! :)\n");
                }
                else{
                    printf("FW update post verified failed: %d :(\n",res);
                }
                updateState(FW_STATE_IDLE);
                break;
            }
            default:{

                vTaskDelay(500);
            }

        }


    }
}

void listFwFiles(){

    for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

        printf("Slot %d: %s (size=%d b) (check=%0x) (design ver=%d)\n",fwFiles[i].fileIndex,fwFileNames[i],fwFiles[i].filesize,fwFiles[i].checksum,fwFiles[i].designver);
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

        lfs_file_t designVerFile = {0};
        char designVerFileName[64]={0};
        sprintf(designVerFileName,"%s.designver",fwFileNames[rx_slot_index]);
        result_fs = fs_file_open(&designVerFile,designVerFileName, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
        if(result_fs <0){
            printf("Could not open file to store design version\n ");
            return -1;
        }
        fs_file_write(&designVerFile, &data->designver, sizeof(uint32_t));
        fs_file_close(&designVerFile);
    }

        return res;
}

int getFwManagerState(){

    return fwMgrState;
}

void fw_mgr_get_rx_progress(int* curr,uint32_t* total ){

    *curr = rx_byte_index;
    *total = fwFiles[rx_slot_index].filesize;

}

int setFwManagerState(int state){

    int res = updateState(state);
    if(res<0){
        printf("Unable to set state!");
    }
    return res;
}

void setFwChecksum(uint8_t slot, uint32_t check){

    Fw_metadata_t metadata;
    memcpy(&metadata,&fwFiles[slot],sizeof(Fw_metadata_t));

    metadata.checksum = check;

    updateFwMetaData(&metadata);


}
void setFwDesignVer(uint8_t slot, uint8_t ver){

    Fw_metadata_t metadata;
    memcpy(&metadata,&fwFiles[slot],sizeof(Fw_metadata_t));

    metadata.designver = ver;

    updateFwMetaData(&metadata);

}

void forceFwManagerState(uint8_t state){

    fwMgrState = state;
}

int updateState(int state){

    int setState = state;

    switch(fwMgrState){

        case FW_STATE_IDLE:{

            //From IDLE we are not allowed to go to UPDATE, otherwise any state is valid.
//            if(state == FW_STATE_UPDATE) return -1;

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

            break;
        }
        case FW_STATE_ARMED:{

            if(state != FW_STATE_IDLE && state != FW_STATE_UPDATE) return -1;

            break;
        }
        case FW_STATE_UPDATE:{

        	//ARMED
            if(state != FW_STATE_IDLE && state != FW_STATE_UPDATE) return -1;
            break;
        }
        case FW_STATE_POST_VERIFY:{

            if(state != FW_STATE_IDLE) return -1;
            break;
        }

    }

    fwMgrState = setState;

    return 0; // TBC: error code?
}

void uploadFwChunk(uint8_t * data, uint16_t length){

    if(length>FW_CHUNK_SIZE){
        printf("fw chunk larger than chunk size!Not ok\n");
        return;
    }
    xQueueSendToBack(fwDataQueue,data,100);
}

void uploadFwChunk2(uint8_t * data, uint16_t length){

    if(length>FW_CHUNK_SIZE){
        printf("fw chunk larger than chunk size!Not ok\n");
        return;
    }
    xQueueSendToBack(fwDataQueue,data,100);
}

void initializeFwMgr(){

    //Reset all our state variables:
    fwMgrState = FW_STATE_IDLE;
    fwMgrArmed = 0;
    fw_armed_timeout=0;
    fwMgrExeConfirmed = 0;
    rx_in_progress = 0;
    rx_slot_index = 0;
    rx_byte_index=0;
    curr_seq_num = 0;
    fw_update_attempts=0;


    //Load/calculate our file metadata
    for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

        lfs_file_t checkFile = {0};
        char checksumFileName[64]={0};
        uint32_t check =0;
        sprintf(checksumFileName,"%s.check",fwFileNames[i%2]);
        int result_fs = fs_file_open(&checkFile,checksumFileName, LFS_O_RDONLY);
        if(result_fs < 0 && result_fs  == LFS_ERR_NOENT){
            printf("Could not open checksum file %s to load: %d\n Creating File...",checksumFileName,result_fs);

            fs_file_open(&checkFile, checksumFileName, LFS_O_CREAT);
            fs_file_close(&checkFile);

        }
        else if (result_fs<0){

           printf("Could not open checksum file %s to load: %d\n",checksumFileName,result_fs);
           continue;
        }
        else{

            if (fs_file_read(&checkFile,&check,sizeof(uint32_t)) != sizeof(uint32_t)){
                printf("Checksum incorrect size\n");
                check = 0;
                //What to do here? I guess leave checksum as 0, which should be impossibly rare to be valid.
            }
        }
        fwFiles[i].checksum = check;
        fs_file_close(&checkFile);

        uint8_t ver = 0xFF;
        lfs_file_t designVerFile = {0};
        char designVerFileName[64]={0};
        sprintf(designVerFileName,"%s.designver",fwFileNames[i%2]);
        result_fs = fs_file_open(&designVerFile,designVerFileName, LFS_O_RDONLY);
        if(result_fs < 0 && result_fs  == LFS_ERR_NOENT){
                    printf("Could not open design ver file %s to load: %d\n Creating File...",designVerFileName,result_fs);

                    fs_file_open(&designVerFile, designVerFileName, LFS_O_CREAT);
                    fs_file_close(&designVerFile);

         }
        else if(result_fs <0){
            printf("Could not open file to read design version\n ");
        }
        else{

            if(fs_file_read(&designVerFile, &ver, sizeof(uint8_t)) != sizeof(uint8_t)){
                printf("Invalid design version read\n");
                ver = 0xFF; //Again nothing really to do here...
            }
        }
        fwFiles[i].designver = ver;
        fs_file_close(&designVerFile);

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

void execute_confirm(){

    if(fwMgrState == FW_STATE_UPDATE){
        fwMgrExeConfirmed = 1;
    }
    else{
        fwMgrExeConfirmed =0;
    }
}

void setFwTargetImage(uint8_t target){

    if(target == 0 || target == 1)
        targetFw = target;
}

void fwRxSendAck(int ackNak, uint16_t seq){

    telemetryPacket_t t;
    Calendar_t now = {0}; //Set to zero, since payload does not keep track of time. CDH will timestamp on receipt.

    uint8_t data[3];

    data[0] = ackNak;
    memcpy(&data[1],&seq,sizeof(uint16_t));

    t.telem_id = CDH_FW_ACK_ID;
    t.timestamp = now;
    t.length = 3;
    t.data = data;

    sendTelemetryAddr(&t, GROUND_CSP_ADDRESS);


}
int fw_mgr_set_arm_timeout(int msec){

    if(msec>1000) fw_user_timeout = msec;

    return fw_user_timeout;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
