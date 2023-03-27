/*
 * cdh.c
 *
 *  Created on: Dec. 6, 2022
 *      Author: jpmckoy
 */

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "application/cdh.h"
#include "main.h"
#include "tasks/fw_update_mgr.h"
#include "drivers/filesystem_driver.h"
#include "drivers/software_update_driver.h"
#include "drivers/device/rtc/rtc_time.h"
#include "drivers/device/memory/flash_common.h"
#include "drivers/protocol/can.h"
#include "drivers/subsystems/eps_driver.h"
#include "tasks/scheduler.h"
#include "application/memory_manager.h"
#include "drivers/device/watchdog.h"

#include "task.h"
#include "version.h"
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
uint8_t can_data_buf[256];
uint8_t tm_id_queue[NUM_MSVB_POLLS_FOR_BACK_SA_PANELS+1] = {POWER_READ_TEMP_ID};
// TODO: mutex backpanel_sa_data usage throughout this file
float backpanel_sa_data[NUM_BACK_SOLAR_STRINGS] = {0.0};
uint8_t backpanel_data_count = 0;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

void HandleCdhCommand(telemetryPacket_t * cmd_pkt)
{
	switch(cmd_pkt->telem_id){
    	case CDH_SCHEDULE_TTT_CMD:{
            uint8_t taskCode = cmd_pkt->data[0];
            uint8_t parameter = cmd_pkt->data[1];
//            Calendar_t * timeTag = (Calendar_t*) (cmd_pkt->data[2]);
//            schedule_task_with_param(taskCode, parameter, *timeTag);
            Calendar_t timeTag = {0};
            memcpy(&timeTag,&cmd_pkt->data[2],sizeof(Calendar_t));
            schedule_task_with_param(taskCode, &parameter, timeTag);
            break;
        }
		case CDH_GET_SPACECRAFT_STATUS_CMD:{
			// Get deployment state
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
			break;
		}
		case CDH_SET_DEPLOYMENT_STARTUP_STATE_CMD:{
			// Set deployment state
			int result;
			uint8_t state = cmd_pkt->data[0];
			result = setDeploymentStartupState(state);
			break;
		}
		case CDH_SET_DETUMBLING_STARTUP_STATE_CMD:{
			// Set detumbling state
			int result;
			uint8_t state = cmd_pkt->data[0];
			result = setDetumblingStartupState(state);
			break;
		}

		default:{
			break;
		}
	} // End of switch(cmd_pkt->telem_id)
} // End of HandleCdhCommand


int handleCdhImmediateCommand(telemetryPacket_t * cmd_pkt, csp_conn_t * conn){

    int result =0;
    switch(cmd_pkt->telem_id)
    {
        case CDH_SET_TIME_CMD:{
            //They send us a Calendar_t
            Calendar_t *newTime = (Calendar_t *) cmd_pkt->data;
            int err = time_valid(newTime);
            if(err == TIME_SUCCESS){
                  //Uncomment for cdh with rtc installed.
//                            ds1393_write_time(newTime);
//                            resync_rtc();
                MSS_RTC_set_calendar_count(newTime);//This is just for testing without actual external rtc. Comment out if using the CDH EM board.
            }else{
                // TBC: Log error...
            }
            break;
        }
        case CDH_GET_TIME_CMD:{
            //They send us a Calendar_t
            Calendar_t currTime;
            MSS_RTC_get_calendar_count(&currTime);
            telemetryPacket_t telem;
            telem.telem_id = CDH_TIME_ID;
            telem.timestamp = currTime;
            telem.length =0;//No data, since the data is in the timestamp.
            telem.data = NULL;
            sendTelemetry_direct(&telem, conn); // TBC: send direct?
            break;
        }
        case GND_TELEMETRY_REQUEST_CMD:{
            TelemetryChannel_t channel_id;
            channel_id = (TelemetryChannel_t) cmd_pkt->data[0];
            get_telemetry(channel_id);
            break;
        }
        case CDH_LIST_FILES_CMD:{
                    fs_list_dir("/",0);
                    break;
                }
                case CDH_LIST_FW_CMD:{
                    listFwFiles();
                    break;
                }
                case CDH_MV_FILE_CMD:{
                    //old_len and new_len should include terminating null.
                    uint8_t old_len = cmd_pkt->data[0];
                    uint8_t new_len = cmd_pkt->data[1];
                    if(old_len<64 && new_len < 64 && old_len+new_len+2 <= cmd_pkt->length){
                        char oldPath[64]={0};
                        char newPath[64]={0};
                        strncpy(oldPath, &cmd_pkt->data[2],old_len);
                        strncpy(newPath, &cmd_pkt->data[2+old_len],new_len);
                        fs_rename(oldPath,newPath);
                    }
                    else{
                        printf("File mv failed, file name(s) too long or discrepancy with telemetry packet length.\n");
                    }
                    break;
                }
                case CDH_RM_FILE_CMD:{
                    if(fs_file_exist(cmd_pkt->data)) fs_remove(cmd_pkt->data);
                    break;
                }
                case CDH_CP_FILE_CMD:{
                    //old_len and new_len should include terminating null.
                    uint8_t old_len = cmd_pkt->data[0];
                    uint8_t new_len = cmd_pkt->data[1];
                    if(old_len<64 && new_len < 64 && old_len+new_len+2 <= cmd_pkt->length){
                        char oldPath[64]={0};
                        char newPath[64]={0};
                        strncpy(oldPath, &cmd_pkt->data[2],old_len);
                        strncpy(newPath, &cmd_pkt->data[2+old_len],new_len);
                        TickType_t start = xTaskGetTickCount();
                        fs_copy_file(oldPath,newPath);
                        TickType_t time = xTaskGetTickCount()-start;
                        printf("copy took %d ms\n",time);
                    }
                    else{
                        printf("File mv failed, file name(s) too long or discrepancy with telemetry packet length.\n");
                    }
                    break;
                }
                case CDH_CHECKSUM_PGRM_FLASH_CMD:{
                    uint32_t* start = (uint32_t*)(&cmd_pkt->data[0]);
                    uint32_t* len = (uint32_t*)(&cmd_pkt->data[sizeof(uint32_t)]);
                    printf("Running checksum of program flash %x for %d bytes\n",*start,*len);
                    //TODO: Document limitation with checksum, cannot be 0. Should be very rare, but pre checksum all files before upload.
                    uint32_t check = 0;
                    checksum_program_flash_area(&check,*start, *len);
                    printf("Checksum of program flash area %d bytes @ 0x%X = %X\n",*len,*start,check);
                    break;
                }
                case CDH_CP_TO_PGRM_FLASH_CMD:{
                    printf("Copy to addr %d from file %s to prog flash\n",*((uint32_t*)(&cmd_pkt->data[0])),&cmd_pkt->data[sizeof(uint32_t)]);
                    copy_to_prog_flash(&cmd_pkt->data[sizeof(uint32_t)], *((uint32_t*)(&cmd_pkt->data[0])));
                    break;
                }
                case CDH_FW_IDLE_CMD:{
                    setFwManagerState(FW_STATE_IDLE);
                    break;
                }
                case CDH_FW_RX_FW_CMD:{
                    if(cmd_pkt->length == sizeof(Fw_metadata_t)){
                        updateFwMetaData((Fw_metadata_t*)cmd_pkt->data);
                    }
                    setFwManagerState(FW_STATE_RX_FW);
                    break;
                }
                case CDH_FW_PRE_VER_CMD:{
                    setFwManagerState(FW_STATE_PRE_VERIFY);
                    break;
                }
                case CDH_FW_ARM_CMD:{
                    setFwManagerState(FW_STATE_ARMED);
                    break;
                }
                case CDH_FW_EXECUTE_CMD:{
                    setFwManagerState(FW_STATE_UPDATE);
                    setFwTargetImage(cmd_pkt->data[0]);
                    break;
                }
                case CDH_FORCE_FW_STATE_CMD:{

                    setFwTargetImage(cmd_pkt->data[1]);
                    forceFwManagerState(cmd_pkt->data[0]);
                    break;
                }
                case CDH_FW_EXECUTE_CONFIRM_CMD:{
                    execute_confirm();
                    break;
                }
                case CDH_FW_POST_VER_CMD:{
                    setFwManagerState(FW_STATE_POST_VERIFY);
                    break;
                }
                case CDH_FW_PUT_DATA_CMD:{
//                    int r = rand();
//                    if(r < 0.99*(double)RAND_MAX){ //10% of time "drop packet". FOR TESTING ONLY! REMOVE!
                        uploadFwChunk(cmd_pkt->data,cmd_pkt->length);
//                    }
//                    else{
//                        printf("CDH Simulated fw packet drop!\n");
//                    }
                    break;
                }
                case CDH_FW_GET_STATE_CMD:{
                    uint8_t state = getFwManagerState();
                    //They send us a Calendar_t
                    Calendar_t currTime;
                    MSS_RTC_get_calendar_count(&currTime);
                    telemetryPacket_t telem;
                    telem.telem_id = CDH_FW_STATE_ID;
                    telem.timestamp = currTime;
                    telem.length =1;
                    telem.data = &state;
                    sendTelemetryAddr(&telem, GROUND_CSP_ADDRESS); // TBC: send direct?
                    break;
                }
                case CDH_CHECKSUM_FILE_CMD:{
                    uint32_t check =0;
                    printf("checksum file: %s\n",cmd_pkt->data);
                    checksum_file(&check, cmd_pkt->data);
                    printf("checksum: %0x\n",check);
                    break;
                }
                case CDH_GET_SW_VER_CMD:{
                    printf("Iris CDH FSW Version: %s",CDH_SW_VERSION_STRING);
                    break;
                }
                case CDH_GET_DES_VER_CMD:{
                    uint16_t dv = get_design_version();
                    printf("Found design ver. %d\n",dv);
                    break;
                }
                case CDH_GET_SPI_DIR_CMD:{
                    uint8_t dir[13]={0};
                    get_spi_dir(dir);
                    for(int i=0;i<13;i++){
                        printf("0x%02X ",dir[i]);
                    }
                    break;
                }
                case CDH_GET_FS_FREE_SPACE_CMD:{
                    uint32_t free = fs_free_space();
                    printf("FS free space = %d bytes\n",free);
                    break;
                }
                case CDH_FW_UPDATE_SPI_DIR_CMD:{
                    //data[0] is the design version of update fw.
                    update_spi_dir(cmd_pkt->data[0], cmd_pkt->data[1]);
                    break;
                }
                case CDH_FW_CREATE_SPI_DIR_CMD:{
                    uint8_t len = cmd_pkt->data[0];
                    create_spi_dir(&cmd_pkt->data[1],len);
                    break;
                }
                case CDH_WRITE_PROG_FLASH_CMD:{
                    uint32_t address=0;
                    memcpy(&address,&cmd_pkt->data[0],sizeof(uint32_t));
                    flash_write(flash_devices[PROGRAM_FLASH], address, &cmd_pkt->data[sizeof(uint32_t)], 128);
        //                      uint8_t check[128]={0};
        //                      flash_read(flash_devices[PROGRAM_FLASH], address, check, 150);
                    break;
                }
                case CDH_ERASE_PROG_FLASH_CMD:{
                    uint32_t address=0;
                    memcpy(&address,&cmd_pkt->data[0],sizeof(uint32_t));
                    uint32_t numblocks = 0;
                    memcpy(&numblocks,&cmd_pkt->data[sizeof(uint32_t)],sizeof(uint32_t));
                    for(int i=0; i< numblocks; i++){
                        flash_erase(flash_devices[PROGRAM_FLASH], address);
                        address += flash_devices[PROGRAM_FLASH]->erase_size;
                    }
                    break;
                }
                case CDH_RESET_FW_MNGR_CMD:{
                    initializeFwMgr();
                    break;
                }
                case CDH_FW_SET_CHECKSUM_CMD:{
                    uint32_t check;
                    memcpy(&check,&cmd_pkt->data[1], sizeof(uint32_t));
                    setFwChecksum(cmd_pkt->data[0], check);
                    break;
                }
                case CDH_FW_SET_DESVER_CMD: {
                    setFwDesignVer(cmd_pkt->data[0], cmd_pkt->data[1]);
                    break;
                }
                case CDH_FORMAT_FS_CMD:{
                    fs_unmount();
                    fs_format();
                    fs_mount();
                    break;
                }
                case CDH_RESET_SYSTEM_CMD:{

                    setLastRebootReason(REBOOT_USER_SOFT_RESET);
                    SCB_Type* systemcontrol = SCB;
                    systemcontrol->AIRCR = (0x05FA << 16)|SCB_AIRCR_SYSRESETREQ_Msk;
                    break;
                }
                case CDH_FW_RX_PROGRESS_CMD:{

                    int curr=0;
                    uint32_t tot=0;
                    fw_mgr_get_rx_progress(&curr,&tot);
                    printf("FW RX: %d bytes of %d\n",curr,tot);

                    break;
                }
                case CDH_FW_PUT_DATA_2_CMD:{
                    //Improved FW Upload.
                    uploadFwChunk2(cmd_pkt->data,cmd_pkt->length);

                    break;
                }
                case CDH_CHECKSUM_FILE_PART_CMD:{

                    uint32_t* start = (uint32_t*)(&cmd_pkt->data[0]);
                    uint32_t* len = (uint32_t*)(&cmd_pkt->data[sizeof(uint32_t)]);
                    char filename[64] ={0};
                    strncpy(filename,(char*)(&cmd_pkt->data[sizeof(uint32_t)*2]),64);

                    printf("Running checksum of file:%s, starting at byte %d for %d bytes\n",filename,*start,*len);
                    //TODO: Document limitation with checksum, cannot be 0. Should be very rare, but pre checksum all files before upload.
                    uint32_t check = 0;
                    checksum_file_area(&check,filename,*start, *len,0);
                    printf("Checksum of file %s, %d bytes @ 0x%X = %X\n",filename,*len,*start,check);

                    break;
                }
                case CDH_FILE_WRITE_CMD:{

                    uint32_t offset=0;
                    uint32_t datalen=0;
                    char filename[64]={0};
                    uint8_t* data;

                    memcpy(&offset,&cmd_pkt->data[0],sizeof(uint32_t));
                    memcpy(&datalen,&cmd_pkt->data[sizeof(uint32_t)],sizeof(uint32_t));
                    strcpy(filename,&cmd_pkt->data[sizeof(uint32_t)*2]);
                    data = &cmd_pkt->data[sizeof(uint32_t)*2+strlen(filename)+1];

                    lfs_file_t file={0};
                    int res= fs_file_open(&file, filename, LFS_O_WRONLY);
                    if(res != FS_OK){
                        printf("Couldn't open file %s to write %d\n.",filename,res);
                        break;
                    }

                    res = fs_file_seek(&file, offset, LFS_SEEK_SET);
                    if(res <= FS_OK){
                        printf("Couldn't seek file %d\n.",res);
                        goto cdh_file_write_end;
                        break;
                    }

                    res = fs_file_write(&file, data, datalen);
                    if(res <= 0){
                        printf("Couldn't write file %d\n.",res);
                        goto cdh_file_write_end;
                        break;
                    }

                cdh_file_write_end:
                    res = fs_file_close(&file);
                    if(res != FS_OK){
                        printf("Couldn't close file %d\n.",res);
                        break;
                    }
                    printf("done\n");
                    break;
                }
                case CDH_FILE_INSERT_CMD:{

                    uint32_t offset=0;
                    uint32_t datalen=0;
                    char filename[64]={0};
                    uint8_t* data;
                    memcpy(&offset,&cmd_pkt->data[0],sizeof(uint32_t));
                    memcpy(&datalen,&cmd_pkt->data[sizeof(uint32_t)],sizeof(uint32_t));
                    strcpy(filename,&cmd_pkt->data[sizeof(uint32_t)*2]);
                    data = &cmd_pkt->data[sizeof(uint32_t)*2+strlen(filename)+1];

                    char filename_temp[64];
                    strcpy(filename_temp,filename);
                    strcat(filename_temp,".tmp");
                    fs_copy_file(filename, filename_temp);

                    lfs_file_t tempfile={0};
                    int res= fs_file_open(&tempfile, filename_temp, LFS_O_WRONLY);
                    if(res != FS_OK){
                        printf("Couldn't open file %s to write %d\n.",filename,res);
                        break;
                    }
                    lfs_file_t file={0};
                    res= fs_file_open(&file, filename, LFS_O_RDONLY);
                    if(res != FS_OK){
                        printf("Couldn't open file %s to read %d\n.",filename,res);
                        break;
                    }

                    //First write the new data to the temp file, at the correct spot.
                    res = fs_file_seek(&tempfile, offset, LFS_SEEK_SET);
                    if(res <= FS_OK){
                        printf("Couldn't seek file %d\n.",res);
                        goto cdh_file_insert_end;
                        break;
                    }
                    res = fs_file_seek(&file, offset, LFS_SEEK_SET);
                    if(res <= FS_OK){
                        printf("Couldn't seek file %d\n.",res);
                        goto cdh_file_insert_end;
                        break;
                    }

                    res = fs_file_write(&tempfile, data, datalen);
                    if(res <= 0){
                        printf("Couldn't write file %d\n.",res);
                        goto cdh_file_insert_end;
                        break;
                    }

                    uint8_t buff[256]={0};
                    int read=0;
                    while((read=fs_file_read(&file,buff,256))>0){

                        int write = fs_file_write(&tempfile,buff,read);
                        if(write != read){
                            printf("error copying file, read != write\n");
                            break;
                        }
                    }



                cdh_file_insert_end:
                    res = fs_file_close(&file);
                    if(res != FS_OK){
                        printf("Couldn't close file %d\n.",res);
                        break;
                    }
                    res = fs_file_close(&tempfile);
                    if(res != FS_OK){
                        printf("Couldn't close file %d\n.",res);
                        break;
                    }
                    res = fs_rename(filename_temp, filename);
                    if(res != FS_OK){
                        printf("Couldn't mv temp file to replace original %d\n.",res);
                        break;
                    }
                    printf("done\n");
                    break;
                }
                case CDH_FILE_DELETE_CMD:{

                    uint32_t offset=0;
                    uint32_t datalen=0;
                    char filename[64]={0};
                    memcpy(&offset,&cmd_pkt->data[0],sizeof(uint32_t));
                    memcpy(&datalen,&cmd_pkt->data[sizeof(uint32_t)],sizeof(uint32_t));
                    strcpy(filename,&cmd_pkt->data[sizeof(uint32_t)*2]);

                    char filename_temp[64];
                    strcpy(filename_temp,filename);
                    strcat(filename_temp,".tmp");
                    fs_copy_file(filename, filename_temp);

                    lfs_file_t tempfile={0};
                    int res= fs_file_open(&tempfile, filename_temp, LFS_O_WRONLY);
                    if(res != FS_OK){
                        printf("Couldn't open file %s to write %d\n.",filename,res);
                        break;
                    }
                    lfs_file_t file={0};
                    res= fs_file_open(&file, filename, LFS_O_RDONLY);
                    if(res != FS_OK){
                        printf("Couldn't open file %s to read %d\n.",filename,res);
                        break;
                    }

                    int filesize = fs_file_size(&tempfile);

                    //Seek so that the read pointer is len bytes ahead, thus deleteing the data between write pointer and read pointer.
                    res = fs_file_seek(&tempfile, offset, LFS_SEEK_SET);
                    if(res <= FS_OK){
                        printf("Couldn't seek file %d\n.",res);
                        goto cdh_file_delete_end;
                        break;
                    }
                    res = fs_file_seek(&file, offset+datalen, LFS_SEEK_SET);
                    if(res <= FS_OK){
                        printf("Couldn't seek file %d\n.",res);
                        goto cdh_file_delete_end;
                        break;
                    }

                    uint8_t buff[256]={0};
                    int read=0;
                    while((read=fs_file_read(&file,buff,256))>0){

                        int write = fs_file_write(&tempfile,buff,read);
                        if(write != read){
                            printf("error copying file, read != write\n");
                            break;
                        }
                    }

                    res = fs_file_truncate(&tempfile, filesize-datalen);
                    if(res != FS_OK){
                        printf("Couldn't truncate file. %d\n.",res);
                        break;
                    }

                cdh_file_delete_end:
                    res = fs_file_close(&file);
                    if(res != FS_OK){
                        printf("Couldn't close file %d\n.",res);
                        break;
                    }
                    res = fs_file_close(&tempfile);
                    if(res != FS_OK){
                        printf("Couldn't close file %d\n.",res);
                        break;
                    }
                    res = fs_rename(filename_temp, filename);
                    if(res != FS_OK){
                        printf("Couldn't mv temp file to replace original %d\n.",res);
                        break;
                    }

                    printf("done\n");

                break;
                }
                case CDH_CLEAR_REBOOT_REASON_CMD:{
                    // Set deployment state
                    int result;
                    uint8_t state = cmd_pkt->data[0];
                    result = setLastRebootReason(state);
                    break;

                    break;
                }
                case CDH_GET_HW_STATUS_CMD:{

                    telemetryPacket_t hwStatPkt = {0};
                    hwStatPkt.telem_id = CDH_HW_STATUS_ID;
                    hwStatPkt.length = sizeof(HardwareCheck_t);
                    hwStatPkt.data = (uint8_t*)&setupHardwareStatus;
                    sendTelemetryAddr(&hwStatPkt, GROUND_CSP_ADDRESS);

                    break;
                }
                case CDH_SET_FW_ARM_TIMEOUT_CMD:{

                    int32_t msec = 0;
                    memcpy(&msec,cmd_pkt->data,sizeof(int32_t));
                    int res =fw_mgr_set_arm_timeout(msec);
                    printf("Timeout: %d\n",res);
                    break;
                }
                case CDH_WATCHDOG_DISABLE_CMD:{

                    uint8_t val = 0;
                    memcpy(&val,cmd_pkt->data,sizeof(uint8_t));
                    start_stop_external_wd(val);

                    break;
                }
                case CDH_RQST_PWR_RESET_LOAD_CMD:{
                	if (cmd_pkt->data[0] >= NUM_LOAD_SWITCHES) {
                		break;
                	} else {
                		resetLoadSwitch(cmd_pkt->data[0]);
                	}
                    break;
                }
        default:{
            result = -1;
            break;
        }
    } // switch(cmd_pkt.telem_id)
    return result;
}

// CAN server variables
extern QueueHandle_t can_rx_queue;
uint8_t numCanMsgs = 0;
CANMessage_t can_q[10] = {0};
uint8_t telem_id = 0;


void unpackRawCanTelemetry(CANMessage_t * can_msg, telemetryPacket_t * output)

{
	memcpy(can_data_buf,0,256);
	memcpy(&output->telem_id,&can_msg->data[0],1);
	memcpy(&output->length,&can_msg->dlc,1);
	output->length -= 1;
	memcpy(can_data_buf,&can_msg->data[1],output->length);
	output->data = can_data_buf;
	MSS_RTC_get_calendar_count(&output->timestamp);
}

void vCanServer(void * pvParameters)
{
	int i;
	CANMessage_t rxmsg = {0};
	telemetryPacket_t tmpkt = {0};
	while(1)
	{
		bool is_backpanel_sa_current;
		if( xQueueReceive(can_rx_queue,&rxmsg,pdMS_TO_TICKS(10000)) )
		{
		 /* rxmsg now contains a copy of xMessage. */
			unpackRawCanTelemetry(&rxmsg, &tmpkt);
			// Update tm_id queue
			is_backpanel_sa_current = true;
			for(i=0; i < NUM_MSVB_POLLS_FOR_BACK_SA_PANELS; i++)
			{
				tm_id_queue[i] = tm_id_queue[i+1];
				if(tm_id_queue[i] != POWER_READ_MSB_VOLTAGE_ID)
				{
					is_backpanel_sa_current = false;
				}
			}
			tm_id_queue[i] = tmpkt.telem_id;
			// Check for back panel solar array data
			if(is_backpanel_sa_current)
			{
				if(backpanel_data_count >= NUM_BACK_SOLAR_STRINGS)
				{
					backpanel_data_count = 0;
				}
				memcpy(&backpanel_sa_data[backpanel_data_count++],tmpkt.data,sizeof(float));
			}

			// TODO: log telemetry
			sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
		}
		vTaskDelay(1100);
	}
}

bool spacecraftIsBackwards(void)
{
	int i;
	for(i=0; i < NUM_BACK_SOLAR_STRINGS; i++)
	{
		if(sa_current_eclipse_threshold < backpanel_sa_data[i])
		{
			return false;
		}
	}
	return true;
}
