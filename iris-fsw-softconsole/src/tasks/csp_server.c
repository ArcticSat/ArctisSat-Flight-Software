//------------------------------------------------------------------------------
// CSP server task and CSP stack setup.
//------------------------------------------------------------------------------
#include "tasks/csp_server.h"
#include "main.h"
#include "tasks/telemetry.h"

#include "csp/csp.h"
#include "csp/interfaces/csp_if_can.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

static uint8_t powerPingStatus = PING_LOST;
static uint8_t powerPingCount = 0;

void sendData(char *buffer, int len, int dest) {
    /* 
    This function sends data over CSP to the specified destination address. 
    It creates a CSP packet, copies the data into it, 
    and sends it through a FreeRTOS queue to the CSP server task for transmission.
    */
    csp_packet_t *newPacket = csp_buffer_get(len);
    satPacket myPacket;

    if (newPacket == NULL) {
        return;
    }

    memcpy(newPacket->data, buffer, len);
    newPacket->length = len;
    myPacket.packet = newPacket;
    myPacket.dest = dest;
    xQueueSendToBack(txQueue, &myPacket, 0);
}

void unpack_2_floats(const uint8_t *buf, float *f1, float *f2) {
    uint32_t u;

    u = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    memcpy(f1, &u, sizeof(float));

    u = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
    memcpy(f2, &u, sizeof(float));
}

void vCSP_Server(void *pvParameters) {
    (void)pvParameters;

    vTaskDelay(1000);
    csp_conn_t *conn = NULL;
    csp_packet_t *packet = NULL;
    csp_socket_t *socket = csp_socket(0);

    // Listen on all ports with a small backlog.
    csp_bind(socket, CSP_ANY);
    csp_listen(socket, 4);

    powerPingStatus = PING_LOST;
    powerPingCount = 0;

    printToTerminal("CSP Server starting\n");
    while (1) {
        conn = csp_accept(socket, 500);
        if (conn != NULL) {
            packet = csp_read(conn, 0);
            int sourceID = csp_conn_src(conn);
            int dest_port = csp_conn_dport(conn);

            // can do some processing based on sourceID and dest_port if needed
            (void)sourceID;
            (void)dest_port;

            if (packet != NULL) {
                // Basic CSP receive demo: just log what came in.
                char logbuf[96];
                int printable = (packet->length < sizeof(packet->data)) ? packet->length : (int)sizeof(logbuf) - 40;
                snprintf(logbuf, sizeof(logbuf), "CSP RX from %d (%d bytes): %.*s\n", sourceID, packet->length, printable, (const char*)packet->data);
                printToTerminal(logbuf);
                csp_buffer_free(packet);
            }
            csp_close(conn);
        }
    }
}

void csp_debug_hook(csp_debug_level_t level, const char *format, va_list args) {
    (void)level;

    // Keep this lightweight to avoid recursive CSP faults.
    char str[256];

    if (0 < vsnprintf(str, sizeof(str) - 1, format, args)) {
        telemetryPacket_t t;
        // Payload does not track time; CDH stamps on receipt.
        Calendar_t now = {0};

        t.telem_id = CDH_MSG_ID;
        t.timestamp = now;
        t.length = strlen(str) + 1;
        t.data = (uint8_t *)str;

        sendTelemetryAddr(&t, GROUND_CSP_ADDRESS);
    }
}

uint8_t configure_csp() {
    // csp_debug_hook_set(&csp_debug_hook);
    csp_debug_set_level(CSP_ERROR, false);
    csp_debug_set_level(CSP_WARN, false);

    struct csp_can_config can_conf = {
        .bitrate = 250000,
        .clock_speed = 250000,
        .ifc = "CAN",
    };

    if (csp_buffer_init(CSP_DEFAULT_NUM_BUFFERS, CSP_DEFAULT_SIZE_BUFFER) != CSP_ERR_NONE) {
        return 0;
    }

    if (csp_init(CDH_CSP_ADDRESS) != CSP_ERR_NONE) {
        return 0;
    }

    if (csp_can_init(CSP_CAN_MASKED, &can_conf) != CSP_ERR_NONE) {
        return 0;
    }

    if (csp_rtable_set(4, 0, &csp_if_can, CSP_NODE_MAC) != CSP_ERR_NONE) {
        return 0;
    }

    if (csp_route_start_task(4 * CSP_DEFAULT_ROUTER_STACK_SIZE, CSP_DEFAULT_ROUTER_PRIORITY) != CSP_ERR_NONE) {
        return 0;
    }

    return 1;
}

