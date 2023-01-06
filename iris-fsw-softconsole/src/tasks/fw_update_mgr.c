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


//Use this define to select if filesystem is used.
#define FW_UPDATE_USE_FS    1

//This is the beginning of where the fw images stored in data flash
#define FW_DATA_BASE_LOCATION   0x00000000

//The max size of a fw image. For full fpga+software it's around 650 kb, so we'll use 1MB.
#define FW_MAX_SIZE     0x100000


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t fwMgrState = FW_STATE_IDLE;
static uint8_t fwMgrArmed = 0;
static uint32_t fw_armed_timeout=0;
static uint8_t fwMgrExeConfirmed = 0;
static uint8_t rx_in_progress = 0;
static uint8_t rx_slot_index = 0;
static uint8_t targetFw = 1;//Which image will we upgrade to. 1 is default for update image. 0 for golden.

static Fw_metadata_t fwFiles[4]; //We keep up to 4 fw. Golden + backup, Upgrade + backup.

static char * fwFileNames[4]={
        "goldenFW.spi",
        "updateFW.spi",
        "goldenFW_bak.spi",
        "updateFW_bak.spi"
};

static uin32_t fw_base_address[5];

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

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int checksumAllFw(){
    int result = 1;

    #if FW_UPDATE_USE_FS

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

    #else

        for(int i=0; i< NUM_FIRMWARES_TOTAL; i++){

        verifiedStatus.verified[i] = 0;

        uint32_t check = 0;
        checksum_program_area(&check,DATA_FLASH,fw_base_address[i],fwFiles[i].size);
        if(check == fwFiles[i].checksum && fwFiles[i].filesize>0 ){
            verifiedStatus.verified[i]=1;
        }
        printf("Firmware %d (%s) verify: %s  (%x, %x)",i,fwFileNames[i],(verifiedStatus.verified[i]?"PASS":"FAIL"),fwFiles[i].checksum,check);

        result &= verifiedStatus.verified[i];
    }

    #endif


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

void checksum_flash_area(uint32_t*out, FlashDevicesIndex_t device , uint32_t address, uint32_t size){

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
    FlashStatus_t stat = flash_read(flash_devices[device],addr_curr, byte_buff, bytes_to_process);

    while( stat == FLASH_OK && bytes_to_process > 0) {

        for(int i =0; i < bytes_to_process; i++){
            crc_32_val = update_crc_32(     crc_32_val, byte_buff[i]);
            count ++;
        }
        addr_curr += bytes_to_process;
        bytes_left = bytes_left-bytes_to_process;
        bytes_to_process = bytes_left>256 ? 256:bytes_left;

        stat = flash_read(flash_devices[device],addr_curr, byte_buff, bytes_to_process);

    }

    crc_32_val        ^= 0xffffffffL;

    *out = crc_32_val;

//    TickType_t time = xTaskGetTickCount()-start;
//    printf("Checksum took %d ms\n",time);
    printf("crc32'd %d bytes\n",count);

}

void checksum_program_flash_area(uint32_t *out,uint32_t address, uint32_t size){

    //Just to avoid redo previous code, just call the new function, hardcocded to use program flash.
    checksum_flash_area(out,PROGRAM_FLASH,address,size);
}

int copy_to_prog_flash(fwMgrDataLocation fw_select, uint32_t address){

    #if FW_UPDATE_USE_FS

        char * filename = fwFileNames[fw_select];
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
    #else

        
        uint32_t filesize = fwFiles[fw_select].size;
        uint32_t chunksize = 256;
        uint8_t byte_buff[256] = {0};
        int wr_addr_curr = address;
        int rd_addr_curr = fw_base_address[fw_select];

        //Erase the area first.
        for(int i=0; i< filesize/4096 + (filesize % 4096 > 0);i++){
            flash_erase(flash_devices[PROGRAM_FLASH],address+(i*4096));
        }

        int bytes_processed = 0;

        while( (bytes_processed < filesize ) ) {
            
            if(bytes_processed + chunksize > filesize) chunksize = filesize-bytes_processed;//Handle the last chunk which will be less than 256.

            flash_read(flash_devices[DATA_FLASH],rd_addr_curr,byte_buff,chunksize)
            flash_write(flash_devices[PROGRAM_FLASH]wr_,addr_curr,byte_buff,chunksize);
            wr_addr_curr += chunksize;
            rd_addr_curr += chunksize;
            bytes_processed += chunksize;
        }

        return 0;


    #endif

}

void flash_copy_file(uint32_t src_addr, uint32_t filesize, uint32_t dst_addr){

        uint32_t chunksize = 256;
        uint8_t byte_buff[256] = {0};
        int wr_addr_curr = dst_addr;
        int rd_addr_curr = src_addr;

        //Erase the area first.
        for(int i=0; i< filesize/4096 + (filesize % 4096 > 0);i++){
            flash_erase(flash_devices[DATA_FLASH],src_addr+(i*4096));
        }

        int bytes_processed = 0;

        while( (bytes_processed < filesize ) ) {
            
            if(bytes_processed + chunksize > filesize) chunksize = filesize-bytes_processed;//Handle the last chunk which will be less than 256.

            flash_read(flash_devices[DATA_FLASH], rd_addr_curr,byte_buff,chunksize)
            flash_write(flash_devices[DATA_FLASH], wr_addr_curr,byte_buff,chunksize);
            wr_addr_curr += chunksize;
            rd_addr_curr += chunksize;
            bytes_processed += chunksize;
        }

}

void handle_fw_upload_no_fs(){


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

                    #if FW_UPDATE_USE_FS
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

                        int write_res =fs_file_write( &fwfile, &rxDataBuff, sizeof(rxDataBuff));
                        if(write_res <0){
                            printf("FwMgr: Problem writing fw chunk to file: %d \n",write_res);
                            updateState(FW_STATE_IDLE);
                            break;
                        }
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
                    #else   
                    handle_fw_upload_no_fs();
                    #endif
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
                        #if FW_UPDATE_USE_FS
                        fs_copy_file(fwFileNames[i+2], fwFileNames[i]);
                        #else   
                        flash_copy_file(fw_base_address[i+2],fwFiles[i+2].size,fw_base_address[i])
                        #endif

                        fwFiles[i].checksum = fwFiles[i+2].checksum;
                        fwFiles[i].filesize = fwFiles[i+2].size;

                    }
                }

                //Now check the opposite, if our backup is missing/bad, then replace with the orig:
                for(int i=0; i<NUM_FIRMWARES; i++){
                    //Backup images are kept sequentially after originals(golden->0, upgrade->1, golden backup->2 etc.)
                    if(verifiedStatus.verified[i+2] ==0 && verifiedStatus.verified[i]==1){
                        //We have a corrupted original so just copy the backup to replace the orig.
                        #if FW_UPDATE_USE_FS
                        fs_copy_file(fwFileNames[i], fwFileNames[i+2]);
                        #else   
                        flash_copy_file(fw_base_address[i],fwFiles[i].size,fw_base_address[i+2])
                        #endif
                        fwFiles[i+2].checksum = fwFiles[i].checksum;
                        fwFiles[i+2].filesize = fwFiles[i].size;
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
                    if(copy_to_prog_flash(0,FIRMWARE_GOLDEN_ADDRESS)){
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
                    if (copy_to_prog_flash(1,FIRMWARE_UPDATE_ADDRESS)<0){
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
                   fw_armed_timeout = FW_ARMED_TIMEOUT_MS;
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

                    //First check our file is verified. We should n't be allowed here without the fw being verified, but just in case...
//                    if(verifiedStatus.verified[1] != 1){
//                        printf("Unverified FW in UPDATE state. This is not allowed, and shouldn't be  possible.\n");
//                        updateState(FW_STATE_IDLE);
//                        break;
//                    }

                    update_spi_dir(targetFw, fwFiles[targetFw].designver);


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


                    //Update state so we know to post verify on reboot.
                    //ShutdownSystem();


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

void initializeFwMgr(){

    //Reset all our state variables:
    fwMgrState = FW_STATE_IDLE;
    fwMgrArmed = 0;
    fw_armed_timeout=0;
    fwMgrExeConfirmed = 0;
    rx_in_progress = 0;
    rx_slot_index = 0;

    #if !FW_UPDATE_USE_FS

    //Initialize the address for storing fw in data flash. Extra slot is for temp file during dowload.
    for(int i=0; i< NUM_FIRMWARES_TOTAL+1; i++){

        fw_base_address[i] = FW_DATA_BASE_LOCATION + i*FW_MAX_SIZE;

    }

    #endif

    load_fw_metadata();

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

void load_fw_metadata(){


#if FW_UPDATE_USE_FS

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

#else



#endif
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
