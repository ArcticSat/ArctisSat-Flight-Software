/*
 * cdh.c
 *
 *  Created on: Dec. 6, 2022
 *      Author: jpmckoy
 */


#include "application/cdh.h"

#include "tasks/fw_update_mgr.h"

#include "drivers/filesystem_driver.h"
#include "drivers/software_update_driver.h"
#include "drivers/device/rtc/rtc_time.h"
#include "drivers/device/memory/flash_common.h"
#include "drivers/protocol/can.h"

#include "task.h"
#include "version.h"

// TBC: change printfs to log_telemetry, with a CDH command option for printing
//		vs



void HandleCdhCommand(telemetryPacket_t * cmd_pkt)
{
	switch(cmd_pkt->telem_id){
    	case CDH_SCHEDULE_TTT_CMD:{
            uint8_t taskCode = cmd_pkt->data[0];
            uint8_t parameter = cmd_pkt->data[1];
            Calendar_t * timeTag = (Calendar_t*) (cmd_pkt->data[2]);
            schedule_task_with_param(taskCode, parameter, *timeTag);
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
		case CDH_FW_EXECUTE_CONFIRM_CMD:{
			execute_confirm();
			break;
		}
		case CDH_FW_POST_VER_CMD:{
			setFwManagerState(FW_STATE_POST_VERIFY);
			break;
		}
		case CDH_FW_PUT_DATA_CMD:{
			uploadFwChunk(cmd_pkt->data,cmd_pkt->length);
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
//                    	uint8_t check[128]={0};
//                    	flash_read(flash_devices[PROGRAM_FLASH], address, check, 150);
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
			SCB_Type* systemcontrol = SCB;
			systemcontrol->AIRCR = (0x05FA << 16)|SCB_AIRCR_SYSRESETREQ_Msk;
			break;
		}
		case CDH_GET_DEPLOYMENT_STARTUP_STATE_CMD:{
			// Get deployment state
			int result;
			uint8_t state;
			result = getDeploymentStartupState(&state);
			// Format data
			uint8_t buf[3] = {0};
			memcpy(buf,&result,sizeof(result));
			memcpy(&buf[2],&state,sizeof(state));
			// Send telemetry packet
			telemetryPacket_t tmpkt = {0};
			tmpkt.telem_id = CDH_DEPLOYMENT_STARTUP_STATE_ID;
			tmpkt.length = 3;
			tmpkt.data = buf;
			sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
			break;
		}
		case CDH_SET_DEPLOYMENT_STARTUP_STATE_CMD:{
			// Set deployment state
			int result;
			uint8_t state = cmd_pkt->data[0];
			result = setDeploymentStartupState(state);
			// Format data
			uint8_t buf[3] = {0};
			memcpy(buf,&result,sizeof(result));
			memcpy(&buf[2],&state,sizeof(state));
			// Send telemetry packet
			telemetryPacket_t tmpkt = {0};
			tmpkt.telem_id = CDH_DEPLOYMENT_STARTUP_STATE_ID;
			tmpkt.length = 3;
			tmpkt.data = buf;
			sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
			break;
		}
		case CDH_GET_DETUMBLING_STARTUP_STATE_CMD:{
			// Get detumbling state
			int result;
			uint8_t state;
			result = getDetumblingStartupState(&state);
			// Format data
			uint8_t buf[3] = {0};
			memcpy(buf,&result,sizeof(result));
			memcpy(&buf[2],&state,sizeof(state));
			// Send telemetry packet
			telemetryPacket_t tmpkt = {0};
			tmpkt.telem_id = CDH_DETUMBLING_STARTUP_STATE_ID;
			tmpkt.length = 3;
			tmpkt.data = buf;
			sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
			break;
		}
		case CDH_SET_DETUMBLING_STARTUP_STATE_CMD:{
			// Set detumbling state
			int result;
			uint8_t state = cmd_pkt->data[0];
			result = setDetumblingStartupState(state);
			// Format data
			uint8_t buf[3] = {0};
			memcpy(buf,&result,sizeof(result));
			memcpy(&buf[2],&state,sizeof(state));
			// Send telemetry packet
			telemetryPacket_t tmpkt = {0};
			tmpkt.telem_id = CDH_DETUMBLING_STARTUP_STATE_ID;
			tmpkt.length = 3;
			tmpkt.data = buf;
			sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
			break;
		}
		default:{
			break;
		}
	} // End of switch(cmd_pkt->telem_id)
} // End of HandleCdhCommand


// CAN server variables
extern QueueHandle_t can_rx_queue;
uint8_t numCanMsgs = 0;
CANMessage_t can_q[10] = {0};
uint8_t telem_id = 0;

void vCanServerBasic(void * pvParameters)
{
	int messages_processed = 0;
	CANMessage_t rx_msg;
	while(1)
	{
		if(numCanMsgs > 0)
		{
			numCanMsgs--;
			uint8_t tm_id = can_q[numCanMsgs].data[0];
			switch(tm_id)
			{
				case POWER_FRAM_GET_OPMODE_ID:{
					telemetryPacket_t telemetry;
					// Send telemetry value
					telemetry.telem_id = POWER_FRAM_GET_OPMODE_ID;
					telemetry.length = 3;
					telemetry.data = &can_q[numCanMsgs].data[1];
					log_telemetry(&telemetry);
					break;
				}
				case POWER_READ_ADC_A_ID:{
					telemetryPacket_t telemetry;
					// Send telemetry value
					telemetry.telem_id = POWER_READ_ADC_A_ID;
					telemetry.length = 2;
					telemetry.data = &can_q[numCanMsgs].data[1];
					log_telemetry(&telemetry);
					break;
				}
				case POWER_READ_ADC_B_ID:{
					telemetryPacket_t telemetry;
					// Send telemetry value
					telemetry.telem_id = POWER_READ_ADC_A_ID;
					telemetry.length = 2;
					telemetry.data = &can_q[numCanMsgs].data[1];
					log_telemetry(&telemetry);
					break;
				}
				default:{
					break;
				}
			}
		}
		vTaskDelay(500);
	}
}


void vCanServer(void * pvParameters)
{
	CANMessage_t rxmsg = {0};
	telemetryPacket_t tmpkt = {0};
	while(1)
	{
		if( xQueueReceive(can_rx_queue,&rxmsg,pdMS_TO_TICKS(10000)) )
		{
		 /* rxmsg now contains a copy of xMessage. */
			unpackRawCanTelemetry(&rxmsg, &tmpkt);
			sendTelemetryAddr(&tmpkt, GROUND_CSP_ADDRESS);
		}

	}
}
