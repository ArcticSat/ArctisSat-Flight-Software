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
#include "application/telemetry_manager.h"
//
#include "drivers/filesystem_driver.h"
#include "drivers/software_update_driver.h"
#include "drivers/device/rtc/rtc_ds1393.h"
#include "drivers/device/rtc/rtc_time.h"
//#include "drivers/device/memory/flash_common.h"

#include "csp/csp.h"
#include "csp/interfaces/csp_if_can.h"
#include "csp/interfaces/csp_if_kiss.h"
#include "drivers/uart_driver_csp.h"
#include "taskhandles.h"
#include "version.h"

#include "FreeRTOS.h"
#include "queue.h"


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

    //Start up any tasks that depend on CSP, FS.
    vTaskResume(vCanServer_h);
    vTaskResume(vTTTScheduler_h);
    //if(get_fs_status() == FS_OK){
    	vTaskResume(vFw_Update_Mgr_Task_h);
    //}

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
					telemetryPacket_t cmd_pkt;
					unpackTelemetry(packet->data, &cmd_pkt);
					switch(cmd_pkt.telem_id)
					{
						case CDH_SET_TIME_CMD:{
							//They send us a Calendar_t
							Calendar_t *newTime = (Calendar_t *) cmd_pkt.data;
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
							channel_id = (TelemetryChannel_t) cmd_pkt.data[0];
							get_telemetry(channel_id);
							break;
						}
				        case CDH_LIST_FW_CMD:{
				            listFwFiles();
				            break;
				        }
				        case CDH_CHECKSUM_PGRM_FLASH_CMD:{
				            uint32_t* start = (uint32_t*)(&cmd_pkt.data[0]);
				            uint32_t* len = (uint32_t*)(&cmd_pkt.data[sizeof(uint32_t)]);
				            printf("Running checksum of program flash %x for %d bytes\n",*start,*len);
				            //TODO: Document limitation with checksum, cannot be 0. Should be very rare, but pre checksum all files before upload.
				            uint32_t check = 0;
				            checksum_program_flash_area(&check,*start, *len);
				            printf("Checksum of program flash area %d bytes @ 0x%X = %X\n",*len,*start,check);
				            break;
				        }
				        case CDH_CP_TO_PGRM_FLASH_CMD:{
				            printf("Copy to addr %d from file %s to prog flash\n",*((uint32_t*)(&cmd_pkt.data[0])),&cmd_pkt.data[sizeof(uint32_t)]);
				            copy_to_prog_flash(&cmd_pkt.data[sizeof(uint32_t)], *((uint32_t*)(&cmd_pkt.data[0])));
				            break;
				        }
				        case CDH_FW_IDLE_CMD:{
				            setFwManagerState(FW_STATE_IDLE);
				            break;
				        }
				        case CDH_FW_RX_FW_CMD:{
				            if(cmd_pkt.length == sizeof(Fw_metadata_t)){
				                updateFwMetaData((Fw_metadata_t*)cmd_pkt.data);
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
				            setFwTargetImage(cmd_pkt.data[0]);
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
				            uploadFwChunk(cmd_pkt.data,cmd_pkt.length);
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
				            printf("checksum file: %s\n",cmd_pkt.data);
				            checksum_file(&check, cmd_pkt.data);
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
				            update_spi_dir(cmd_pkt.data[0], cmd_pkt.data[1]);
				            break;
				        }
				        case CDH_FW_CREATE_SPI_DIR_CMD:{
				            uint8_t len = cmd_pkt.data[0];
				            create_spi_dir(&cmd_pkt.data[1],len);
				            break;
				        }
				        case CDH_WRITE_PROG_FLASH_CMD:{
				            uint32_t address=0;
				            memcpy(&address,&cmd_pkt.data[0],sizeof(uint32_t));
				            flash_write(flash_devices[PROGRAM_FLASH], address, &cmd_pkt.data[sizeof(uint32_t)], 128);
				//                      uint8_t check[128]={0};
				//                      flash_read(flash_devices[PROGRAM_FLASH], address, check, 150);
				            break;
				        }
				        case CDH_ERASE_PROG_FLASH_CMD:{
				            uint32_t address=0;
				            memcpy(&address,&cmd_pkt.data[0],sizeof(uint32_t));
				            uint32_t numblocks = 0;
				            memcpy(&numblocks,&cmd_pkt.data[sizeof(uint32_t)],sizeof(uint32_t));
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
				            memcpy(&check,&cmd_pkt.data[1], sizeof(uint32_t));
				            setFwChecksum(cmd_pkt.data[0], check);
				            break;
				        }
				        case CDH_FW_SET_DESVER_CMD: {
				            setFwDesignVer(cmd_pkt.data[0], cmd_pkt.data[1]);
				            break;
				        }
						default:{
							schedule_command(&cmd_pkt);
							break;
						}
					} // switch(cmd_pkt.telem_id)
					break;
				} // case CSP_CMD_PORT
				case CSP_TELEM_PORT:
					default:{
						csp_service_handler(conn,packet);
						break;
					}
				} // case CSP_TELEM_PORT
				//Should buffer free be here? Example doesn't call this after csp_service handler.
				csp_buffer_free(packet);
				csp_close(conn);
		} // if(conn)
	} // while(1)
} // End of vCSP_Server

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
    status = csp_init(CDH_CSP_ADDRESS);
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
