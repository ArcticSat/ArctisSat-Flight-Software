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

//------------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------------

void vCSP_Server(void *pvParameters) {

    uint8_t result = configure_csp();
    if (result) {
        set_csp_init(1);
    }

    csp_conn_t *conn = NULL;
    csp_packet_t *packet = NULL;
    csp_socket_t *socket = csp_socket(0);

    csp_bind(socket, CSP_ANY); //bind to all ports, listen to anything set to this destination address
    csp_listen(socket, 4); //have up to 4 backlog connections for testing
    vTaskResume(vCanServer_h); //resume CAN rx handler

    while (1) {
        conn = csp_accept(socket, 1000); //1000ms timeout, avoid starving lower priority tasks
        if (conn) {
            packet = csp_read(conn, 0); //connection was made, read received packet
            int dest_port = csp_conn_dport(conn);
            csp_buffer_free(packet); //remove packet from heap
            csp_close(conn); //close connection
        } // if(conn)
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

