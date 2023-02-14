//------------------------------------------------------------------------------
// MBSat-1
//
// Repository:
//	Github: https://github.com/joehowarth17/ManitobaSat-Flight-Software
//
// File Description:
//  This file contains the Cubesat Space Protocol server.
//
// History
// 2020-04-10 by Joseph Howarth
// - Created.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "tasks/csp_server.h"
#include "tasks/telemetry.h"
#include "tasks/scheduler.h"
#include "tasks/fw_update_mgr.h"

#include "drivers/filesystem_driver.h"
#include "drivers/software_update_driver.h"
#include "drivers/device/rtc/rtc_ds1393.h"
#include "drivers/device/rtc/rtc_time.h"
#include "drivers/device/memory/flash_common.h"

#include "csp/csp.h"
#include "csp/interfaces/csp_if_can.h"
#include "csp/interfaces/csp_if_kiss.h"
#include "drivers/uart_driver_csp.h"
#include "taskhandles.h"
#include "version.h"

#include "FreeRTOS.h"
#include "queue.h"

#define USING_EM_OR_FM 0


//------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//  Description: This function configures CSP. It handles setting up buffers,
//               initializing the interface(s), setting up the router table,
//               and starting the router task. The defualt values are listed
//               in the header file.
//
// Returns:     Returns 1 if sucessful, 0 if there is an error.
//------------------------------------------------------------------------------
uint8_t configure_csp();

//------------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------------

void vCSP_Server(void * pvParameters){

    InputQueues_t * queues = (InputQueues_t *) pvParameters;

    uint8_t result = configure_csp();

    csp_conn_t * conn = NULL;
	csp_packet_t * packet= NULL;
	csp_socket_t * socket = csp_socket(0);

    //Listen for messages to all ports.
    csp_bind(socket, CSP_ANY);

    //Have up to 4 backlog connections.
    csp_listen(socket,4);

#if USING_EM_OR_FM
    int result_fs = 1;
    uint32_t boot_count = 0;

    lfs_file_t file = {0}; //Set to 0 because debugger tries to read fields of struct one of which is a pointer, but since this is on free rtos heap, initial value is a5a5a5a5.

    FilesystemError_t stat = fs_init();
    if(stat != FS_OK){
        while(1){}
    }
    //Mount the file system.
    int err = fs_mount();

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        fs_format(); //Is there anything else we should do or try first?
        fs_mount();
    }
   result_fs = fs_file_open( &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
   if(result_fs < 0) while(1){}

   result_fs = fs_file_read( &file, &boot_count, sizeof(boot_count));
   if(result_fs < 0) while(1){}

   // update boot count
   boot_count += 1;
   result_fs = fs_file_rewind( &file);
   if(result_fs < 0) while(1){}

   result_fs = fs_file_write( &file, &boot_count, sizeof(boot_count));
   if(result_fs < 0) while(1){}

   // remember the storage is not updated until the file is closed successfully
   result_fs = fs_file_close( &file);
   if(result_fs < 0) while(1){}

   printf("CDH has started for the %dth time\n",boot_count);//Log this instead or send as telem.

   result_fs = fs_file_open( &file, "test_file.txt", LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC);
  if(result_fs < 0) while(1){}

  result_fs = fs_file_write( &file, "hello world", strlen("hello world"));
  if(result_fs < 0) while(1){}
  result_fs = fs_file_close( &file);
  if(result_fs < 0) while(1){}

#endif

//    //Start up any tasks that depend on CSP.
//    vTaskResume(vTestCanServer_h);
//    vTaskResume(vFw_Update_Mgr_Task_h);
//    vTaskResume(vTTTScheduler_h);
  	  CANMessage_t cmd = {0};
  	cmd.id = POWER_CAN_ID;
  	int i;

    //TODO: Check return of csp_bind and listen, then handle errors.
    while(1) {

		conn = csp_accept(socket, 1000);

		if(conn){

			packet = csp_read(conn,0);

            #ifdef DEBUG
                //prvUARTSend(&g_mss_uart0, packet->data, packet->length);
			#endif

            //Handle the message based on the port it was sent to.
            int dest_port = csp_conn_dport(conn);

            switch(dest_port){

            case CSP_CMD_PORT:{

                    telemetryPacket_t t ;
                    unpackTelemetry(packet->data, &t);

                    memcpy(cmd.data,0,8);

                    switch(t.telem_id){

                    case POWER_READ_TEMP_CMD:{ // Read temperature value command
						cmd.dlc = 2;
						cmd.data[0] = POWER_READ_TEMP_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_READ_SOLAR_CURRENT_CMD:{ // Read solar current command
						cmd.dlc = 2;
						cmd.data[0] = POWER_READ_SOLAR_CURRENT_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_READ_LOAD_CURRENT_CMD:{ // Read solar current command
						cmd.dlc = 2;
						cmd.data[0] = POWER_READ_LOAD_CURRENT_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_READ_MSB_VOLTAGE_CMD:{ // Read solar current command
						cmd.dlc = 1;
						cmd.data[0] = POWER_READ_MSB_VOLTAGE_CMD;
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_GET_BATTERY_SOC_CMD:{ // Read solar current command
						cmd.dlc = 1;
						cmd.data[0] = POWER_GET_BATTERY_SOC_CMD;
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_GET_SA_CHARGE_STATE_CMD:{ // Read solar current command
						cmd.dlc = 1;
						cmd.data[0] = POWER_GET_SA_CHARGE_STATE_CMD;
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_GET_BOOT_COUNT_CMD:{ // Read solar current command
						cmd.dlc = 1;
						cmd.data[0] = POWER_GET_BOOT_COUNT_CMD;
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_SET_LOAD_OFF_CMD:{
						cmd.dlc = 2;
						cmd.data[0] = POWER_SET_LOAD_OFF_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_SET_LOAD_ON_CMD:{
						cmd.dlc = 2;
						cmd.data[0] = POWER_SET_LOAD_ON_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_SET_SOLAR_OFF_CMD:{
						cmd.dlc = 2;
						cmd.data[0] = POWER_SET_SOLAR_OFF_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_SET_SOLAR_ON_CMD:{
						cmd.dlc = 2;
						cmd.data[0] = POWER_SET_SOLAR_ON_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_SET_POW_MODE_CMD:{
						cmd.dlc = 2;
						cmd.data[0] = POWER_SET_POW_MODE_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_AIT_SET_BATTERY_SOC_CMD:{
						float soc;
						memcpy(&soc,t.data,sizeof(float));
						cmd.dlc = 5;
						cmd.data[0] = POWER_AIT_SET_BATTERY_SOC_CMD;
						memcpy(&cmd.data[1],&t.data[0],sizeof(float));
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_FRAM_GET_OPMODE_CMD: {
						cmd.dlc = 1;
						cmd.data[0] = POWER_FRAM_GET_OPMODE_CMD;
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_FRAM_GET_SOC_CMD: {
						cmd.dlc = 1;
						cmd.data[0] = POWER_FRAM_GET_SOC_CMD;
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_FRAM_GET_BOOT_COUNT_CMD: {
						cmd.dlc = 1;
						cmd.data[0] = POWER_FRAM_GET_BOOT_COUNT_CMD;
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_FRAM_LOG_OPMODE_CMD:{
						cmd.dlc = 2;
						cmd.data[0] = POWER_FRAM_LOG_OPMODE_CMD;
						cmd.data[1] = t.data[0];
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_FRAM_LOG_SOC_CMD:{
						float soc;
						memcpy(&soc,t.data,sizeof(float));
						cmd.dlc = 5;
						cmd.data[0] = POWER_FRAM_LOG_SOC_CMD;
						memcpy(&cmd.data[1],t.data,sizeof(float));
//						memcpy(&cmd.data[1],&t.data[0],sizeof(float));
						CAN_transmit_message(&cmd);
						break;
					}
					case POWER_FRAM_LOG_BOOT_COUNT_CMD:{
						uint16_t count;
						memcpy(&count,t.data,sizeof(count));
						cmd.dlc = 3;
						cmd.data[0] = POWER_FRAM_LOG_BOOT_COUNT_CMD;
						memcpy(&cmd.data[1],t.data,sizeof(count));
						CAN_transmit_message(&cmd);
						break;
					}

                    case CDH_SCHEDULE_TTT_CMD:{

                            uint8_t taskCode = t.data[0];
                            uint8_t parameter = t.data[1];
                            Calendar_t timeTag = *((Calendar_t*)&t.data[2]);

                            schedule_task_with_param(taskCode, parameter, timeTag);
                            break;
                        }

                    case CDH_SET_TIME_CMD:{

                        //They send us a Calendar_t
                        Calendar_t *newTime = t.data;
                        int err = time_valid(newTime);

                        if(err == TIME_SUCCESS){
                              //Uncomment for cdh with rtc installed.
//                            ds1393_write_time(newTime);
//                            resync_rtc();
                            MSS_RTC_set_calendar_count(newTime);//This is just for testing without actual external rtc. Comment out if using the CDH EM board.
                        }else{

                            //Log error...
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

                        sendTelemetry_direct(&telem, conn);
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
                        uint8_t old_len = t.data[0];
                        uint8_t new_len = t.data[1];

                        if(old_len<64 && new_len < 64 && old_len+new_len+2 <= t.length){
                            char oldPath[64]={0};
                            char newPath[64]={0};

                            strncpy(oldPath, &t.data[2],old_len);
                            strncpy(newPath, &t.data[2+old_len],new_len);
                            fs_rename(oldPath,newPath);
                        }
                        else{
                            printf("File mv failed, file name(s) too long or discrepancy with telemetry packet length.\n");
                        }
                        break;
                    }
                    case CDH_RM_FILE_CMD:{

                        if(fs_file_exist(t.data)) fs_remove(t.data);

                        break;

                        break;
                    }
                    case CDH_CP_FILE_CMD:{

                        //old_len and new_len should include terminating null.
                        uint8_t old_len = t.data[0];
                        uint8_t new_len = t.data[1];

                        if(old_len<64 && new_len < 64 && old_len+new_len+2 <= t.length){
                            char oldPath[64]={0};
                            char newPath[64]={0};

                            strncpy(oldPath, &t.data[2],old_len);
                            strncpy(newPath, &t.data[2+old_len],new_len);

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

                        uint32_t* start = (uint32_t*)(&t.data[0]);
                        uint32_t* len = (uint32_t*)(&t.data[sizeof(uint32_t)]);
                        printf("Running checksum of program flash %x for %d bytes\n",*start,*len);
                        //TODO: Document limitation with checksum, cannot be 0. Should be very rare, but pre checksum all files before upload.
                        uint32_t check = 0;
                        checksum_program_flash_area(&check,*start, *len);
                        printf("Checksum of program flash area %d bytes @ 0x%X = %X\n",*len,*start,check);
                        break;
                    }
                    case CDH_CP_TO_PGRM_FLASH_CMD:{

                        printf("Copy to addr %d from file %s to prog flash\n",*((uint32_t*)(&t.data[0])),&t.data[sizeof(uint32_t)]);
                        copy_to_prog_flash(&t.data[sizeof(uint32_t)], *((uint32_t*)(&t.data[0])));

                        break;
                    }
                    case CDH_FW_IDLE_CMD:{

                        setFwManagerState(FW_STATE_IDLE);
                        break;
                    }
                    case CDH_FW_RX_FW_CMD:{

                        if(t.length == sizeof(Fw_metadata_t)){
                            updateFwMetaData((Fw_metadata_t*)t.data);
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
                        setFwTargetImage(t.data[0]);

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

                        uploadFwChunk(t.data,t.length);

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

                        sendTelemetryAddr(&telem, GROUND_CSP_ADDRESS);
                        break;
                    }
                    case CDH_CHECKSUM_FILE_CMD:{

                        uint32_t check =0;
                        printf("checksum file: %s\n",t.data);
                        checksum_file(&check, t.data);
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
                    	update_spi_dir(t.data[0], t.data[1]);
                    	break;
                    }
                    case CDH_FW_CREATE_SPI_DIR_CMD:{

                    	uint8_t len = t.data[0];
                    	create_spi_dir(&t.data[1],len);

                    	break;
                    }
                    case CDH_WRITE_PROG_FLASH_CMD:{

                    	uint32_t address=0;
                    	memcpy(&address,&t.data[0],sizeof(uint32_t));

                    	flash_write(flash_devices[PROGRAM_FLASH], address, &t.data[sizeof(uint32_t)], 128);
//                    	uint8_t check[128]={0};
//                    	flash_read(flash_devices[PROGRAM_FLASH], address, check, 150);
                    	break;
                    }
                    case CDH_ERASE_PROG_FLASH_CMD:{
                    	uint32_t address=0;
                    	memcpy(&address,&t.data[0],sizeof(uint32_t));
                    	uint32_t numblocks = 0;
                    	memcpy(&numblocks,&t.data[sizeof(uint32_t)],sizeof(uint32_t));
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
                        memcpy(&check,&t.data[1], sizeof(uint32_t));

                        setFwChecksum(t.data[0], check);

                        break;
                    }
                    case CDH_FW_SET_DESVER_CMD: {

                        setFwDesignVer(t.data[0], t.data[1]);


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

                    break;
                }
                break;
            }
            case CSP_TELEM_PORT:

                    break;

                default:
                    csp_service_handler(conn,packet);
                    break;
            }
            //Should buffer free be here? Example doesn't call this after csp_service handler.
            csp_buffer_free(packet);
			csp_close(conn);
		}
	}
}

uint8_t configure_csp(){

    csp_debug_set_level(CSP_ERROR, false);
    uint8_t result = 1; //Sucess
    // CAN parameters are not actually used. Need to decide where we are doing
    // CAN init. Right now the csp driver does this, but uses hard coded params.
    struct csp_can_config can_conf;
    can_conf.bitrate=250000;
    can_conf.clock_speed=250000;
    can_conf.ifc = "CAN";

    /* Init buffer system with 5 packets of maximum 256 bytes each */
    int status = csp_buffer_init(CSP_DEFAULT_NUM_BUFFERS, CSP_DEFAULT_SIZE_BUFFER);
    if(status != CSP_ERR_NONE){
        result = 0;
        return result;
    }
    /* Init CSP with address 0 */
    status = csp_init(GROUND_CSP_ADDRESS2);
    if(status != CSP_ERR_NONE){
        result = 0;
        return result;
    }

    /* Init the CAN interface with hardware filtering */
    status = csp_can_init(CSP_CAN_MASKED, &can_conf);
    if(status != CSP_ERR_NONE){
        result = 0;
        return result;
    }

    csp_kiss_init(&uartInterface, &uartHandle, uartPutChar, NULL, "KISS");

    /* Setup default route to CAN interface */
    //status = csp_rtable_set(CSP_DEFAULT_ROUTE,0, &csp_if_can,CSP_NODE_MAC);
    char* canRoute = "0/2 CAN";
    char* gndRoute = "9/5 KISS";

   csp_rtable_load(canRoute);
   csp_rtable_load(gndRoute);
    if(status != CSP_ERR_NONE){
        result = 0;
        return result;
    }

    /* Start router task with 100 word stack, OS task priority 1 */
    status = csp_route_start_task(4*CSP_DEFAULT_ROUTER_STACK_SIZE, CSP_DEFAULT_ROUTER_PRIORITY);
    if(status != CSP_ERR_NONE){
        result = 0;
        return result;
    }

    return result;
}
