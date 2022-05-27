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

    //Run the state machine.
    while(1){

        switch(fwMgrState){

            case FW_STATE_IDLE:{

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
                                int exist = fs_file_open(&fwfile, fwFileNames[rx_slot_index], LFS_O_RDONLY);
                                fs_file_close(&fwfile);
                                if(exist >=0){
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

                                exist = fs_file_open(&fwfile, fwFileNames[rx_slot_index], LFS_O_RDONLY);
                                fs_file_close(&fwfile);
                                if(exist >=0){
                                    res = fs_remove(tempFileName);
                                    if(res<0){
                                        printf("FwMgr: Could not finish uploading fw, cant remove temp: %d\n",res);
                                        updateState(FW_STATE_IDLE);
                                        break;
                                    }
                                }
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

                //Start by checksuming the two files.


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


}

int updateFwMetaData(Fw_metadata_t* data){

    int res = -1;
    if(data->fileIndex <= 1 && data->fileIndex >= 0){
        memcpy(&fwFiles[data->fileIndex],data, sizeof(Fw_metadata_t));
        res = 0;
        rx_in_progress = 1;
        rx_slot_index = data->fileIndex;
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
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
