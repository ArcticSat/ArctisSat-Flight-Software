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
#include "main.h"
#include "tasks/telemetry.h"
#include "tasks/scheduler.h"
//#include "tasks/fw_update_mgr.h"
#include "application/cdh.h"
#include "application/memory_manager.h"

#include "drivers/filesystem_driver.h"
//#include "drivers/software_update_driver.h"
#include "drivers/device/rtc/rtc_ds1393.h"
#include "drivers/device/rtc/rtc_time.h"
//#include "drivers/device/memory/flash_common.h"
#include "application/application.h"

#include "csp/csp.h"
#include "csp/interfaces/csp_if_can.h"
#include "csp/interfaces/csp_if_kiss.h"
#include "drivers/uart_driver_csp.h"
#include "taskhandles.h"
#include "drivers/protocol/uart.h"

#include "tasks/healthAndSafety.h"

//#include "version.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

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

void sendData(char* buffer, int len, int dest) {
    csp_packet_t *newPacket = csp_buffer_get(len); // Get a buffer large enough to fit our data. Max size is 256.
    satPacket myPacket;
    if(newPacket) {
//              test = &CCLSM_DATA[i];
        memcpy(newPacket->data, buffer, len);
        newPacket->length = len;
        myPacket.packet = newPacket;
        myPacket.dest = dest;
        xQueueSendToBack(txQueue, &myPacket, 0);
    }
}

//------------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------------

int lossLockout = 0;
uint8_t powerPingStatus = PING_LOST;
uint8_t powerPingCount = 0;

void unpack_2_floats(const uint8_t *buf, float *f1, float *f2)
{
    uint32_t u;

    u = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    memcpy(f1, &u, sizeof(float));

    u = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
    memcpy(f2, &u, sizeof(float));
}


void vCSP_Server(void *pvParameters) {
    vTaskDelay(1000);
    csp_conn_t *conn = NULL;
    csp_packet_t *packet = NULL;
    csp_socket_t *socket = csp_socket(0);

    csp_bind(socket, CSP_ANY); //bind to all ports, listen to anything set to this destination address
    csp_listen(socket, 4); //have up to 4 backlog connections for testing
//    vTaskResume(vCanServer_h); //resume CAN rx handler

    //Have up to 4 backlog connections.
//    csp_listen(socket,4);

    powerPingStatus = PING_LOST;
    powerPingCount = 0;

    static CCLSM_DATA_ENTRY testCCLSM;
    
    int misses = 0;
    int lockout = 0;
    float msbVoltage;
    float msbCurrent;
    volatile char testBuf[64];
    //TODO: Check return of csp_bind and listen, then handle errors.
    //TODO make this so much better!
    printToTerminal("CSP Server starting\n");
    while(1) {
		conn = csp_accept(socket, 500);
		if(conn){
            packet = csp_read(conn, 500);
            if (packet == NULL) {
                csp_close(conn);
                continue;
            }
			int sourceID = csp_conn_src(conn);
            int dest_port = csp_conn_dport(conn);

			switch(sourceID){
                case 0x00: //power ID
                {
                    powerPingCount = 0;
                    if(lockout) {
                        logError(WARN_POWER_COMMS_RESTORED, SEV_WARNING, NULL, 0);
                        // logPowerTelem("Power restored!\n", strlen("Power restored!\n")+1);
                    }
                    lockout = 0;
                    powerPingStatus = PING_FOUND;

                    break;
                }

                case 0x01: //payload
                {
                    telemetryPacket_t respPkt;
                    if (packet->length >= sizeof(respPkt)) {
                      memcpy(testBuf, packet->data, packet->length);
                      volatile int breakpoint = 1;
                    }
                }
			}

			switch(dest_port){
			    case 5: {
                    sendRawData((char*)packet->data, 0x01, packet->length);
                    // unpack_2_floats(packet->data, &msbVoltage, &msbCurrent);

                break;
                }

				default:{
						csp_service_handler(conn,packet);
						break;
					}
				} // case CSP_TELEM_PORT
				//Should buffer free be here? Example doesn't call this after csp_service handler.
                csp_buffer_free(packet);
				csp_close(conn);
		} else {
		    powerPingCount++;
		}

		if(powerPingCount > 5) {
		    powerPingStatus = PING_LOST;
		    powerPingCount = 5;
            if(lockout == 0){
                lockout = 1;
                logPowerTelem("Power lost!\n", strlen("Power lost!\n")+1);
                logError(ERR_POWER_LOST, SEV_CRITICAL, NULL, 0);
            }
        }
//		vTaskDelay(500);
	} // while(1)
} // End of vCSP_Server

void csp_debug_hook(csp_debug_level_t level, const char *format, va_list args) {
    //Only for debug purpose! We must be careful because this function
    // is called when there is a problem with csp, but then we go and use csp to log the error...
    //Can lead to stack overflow if the error being logged is critical, like running out of connections or buffer.
    //Main point is to get an idea of less critical error or warning and to avoid the csp_sys_set_color() call in the default debug handler.
    //We should instead log the csp errors to a file and can hopefully then recover csp and download the file to learn more.
    char str[256];

    if (0 < vsnprintf(str, 255, format, args)) // build string
            {
        telemetryPacket_t t;
        Calendar_t now = { 0 }; //Set to zero, since payload does not keep track of time. CDH will timestamp on receipt.

        //      t.telem_id = PAYLOAD_ERROR_ID;
        t.telem_id = CDH_MSG_ID;
        t.timestamp = now;
        t.length = strlen(str) + 1;
        t.data = (uint8_t*) str;

        sendTelemetryAddr(&t, GROUND_CSP_ADDRESS);
    }

}

uint8_t configure_csp() {

    //csp_debug_hook_set(&csp_debug_hook);
    csp_debug_set_level(CSP_ERROR, false);
    csp_debug_set_level(CSP_WARN, false);

    uint8_t result = 1; //Sucess
    // CAN parameters are not actually used. Need to decide where we are doing
    // CAN init. Right now the csp driver does this, but uses hard coded params.
    struct csp_can_config can_conf;
    can_conf.bitrate = 250000;
    can_conf.clock_speed = 250000;
    can_conf.ifc = "CAN";

    /* Init buffer system with 5 packets of maximum 256 bytes each */
    int status = csp_buffer_init(CSP_DEFAULT_NUM_BUFFERS, CSP_DEFAULT_SIZE_BUFFER);
    if (status != CSP_ERR_NONE) {
        result = 0;
        return result;
    }
    /* Init CSP with address 0 */
    status = csp_init(CDH_CSP_ADDRESS);
    if (status != CSP_ERR_NONE) {
        result = 0;
        return result;
    }

    /* Init the CAN interface with hardware filtering */
    status = csp_can_init(CSP_CAN_MASKED, &can_conf);
    if (status != CSP_ERR_NONE) {
        result = 0;
        return result;
    }

    /* Setup default route to CAN interface */
    status = csp_rtable_set(4, 0, &csp_if_can, CSP_NODE_MAC);
    if (status != CSP_ERR_NONE) {
        result = 0;
        return result;
    }

    /* Start router task with 100 word stack, OS task priority 1 */
    status = csp_route_start_task(4 * CSP_DEFAULT_ROUTER_STACK_SIZE, CSP_DEFAULT_ROUTER_PRIORITY);
    if (status != CSP_ERR_NONE) {
        result = 0;
        return result;
    }

    return result;
}

