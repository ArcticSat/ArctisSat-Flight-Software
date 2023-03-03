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
#include "application/memory_manager.h"
#include "application/sc_deployment.h"
#include "tasks/csp_server.h"
#include "tasks/telemetry.h"
#include "tasks/scheduler.h"
//#include "tasks/fw_update_mgr.h"
#include "drivers/filesystem_driver.h"
//#include "drivers/software_update_driver.h"
#include "drivers/device/rtc/rtc_ds1393.h"
#include "drivers/device/rtc/rtc_time.h"
//#include "drivers/device/memory/flash_common.h"

#include "csp/csp.h"
#include "csp/interfaces/csp_if_can.h"
#include "csp/interfaces/csp_if_kiss.h"
#include "drivers/uart_driver_csp.h"
#include "taskhandles.h"
//#include "version.h"

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

	//Make sure FS is up before all tasks
	filesystem_initialization();

	// Initialize the spacecraft's status
	int result_fs;
	result_fs = InitSpacecraftStatus();

	// Check deployment state
	uint8_t deployment_state;
	getDeploymentStartupState(&deployment_state);
	if(deployment_state == DPL_STATE_STOWED)
	{
		InitiateSpacecraftDeployment();
	}

    // Start up any tasks that depend on CSP, FS.
	uint8_t detumble_state;
	result_fs = getDetumblingStartupState(&detumble_state);
	if(result_fs == FS_OK && detumble_state == DETUMBLING_NOT_COMPLETE)
	{
		// Detumble mode
		vTaskResume(vDetumbleDriver_h);
	}
	else
	{
		// Normal operations
		vTaskResume(vCanServer_h);
		vTaskResume(vTTTScheduler_h);
	//    if(get_fs_status() == FS_OK){
	//    	vTaskResume(vFw_Update_Mgr_Task_h);
	//    }
	}


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

//    csp_kiss_init(&uartInterface, &uartHandle, uartPutChar, NULL, "KISS");

    /* Setup default route to CAN interface */
    //status = csp_rtable_set(CSP_DEFAULT_ROUTE,0, &csp_if_can,CSP_NODE_MAC);
    char* canRoute = "0/0 CAN";

//    char* gndRoute = "9/5 KISS";

   csp_rtable_load(canRoute);
//   csp_rtable_load(gndRoute);
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
