/*
 * comms_handler.c
 *
 *  Created on: Jan 23, 2025
 *      Author: brend
 */

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"
#include "tasks/csp_server.h"

#include <string.h>
#include <csp/csp.h>
#include "csp/interfaces/csp_if_can.h"
#include "drivers/protocol/uart.h"
#include "tasks/telemetry.h"
#include "application/adcs.h"
#include "application/application.h"

uint32_t crc32b(unsigned char *data, int size) {
    int byteIdx, bitIdx;
    uint32_t crc = 0xFFFFFFFF;
    for (byteIdx = 0; byteIdx < size; byteIdx++) {
        char ch = data[byteIdx];
        for (bitIdx = 0; bitIdx < 8; bitIdx++) {
            uint32_t b = (ch ^ crc) & 1;
            crc >>= 1;
            if (b)
                crc = crc ^ 0x04C11DB7;
            ch >>= 1;
        }
    }
    return ~crc;
}

volatile int CTS = 1;
static TaskHandle_t xTaskToNotify = NULL;

void sendImagePacket(char *data, int len, int index) {
    int currLen = len;
    radioPacket_t packet;
    packet.header = 0xAA;
    packet.footer = 0xBB;
    packet.index = index;
    packet.len = len;
    memcpy(packet.data, data, len);
    packet.type = 0x99;
    uint32_t tempCRC = crc32b((char*) &packet, 64 + 6);
    packet.crc = tempCRC;
    xQueueSendToBack(commsTxQueue, &packet, portMAX_DELAY);
}

void commsTransmitterTask() {
    radioPacket_t *packet;
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    uint32_t *notVal;

//    vTaskDelay(50000);
    printToTerminal("COMMS Transmitter task started.\n");
    for (;;) {
        // xTaskNotifyWait(0, 0, notVal, portMAX_DELAY);
        if (xQueueReceive(commsTxQueue, &packet, portMAX_DELAY) == pdTRUE) {
//            custom_MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t*) "[", 1);
//            custom_MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t*) &packet->headerStr, strlen((char*)packet->headerStr));
//            custom_MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t*) "] ", 2);
            custom_MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t*) &packet->data, packet->len);
            vPortFree(packet);
        }
    }
}

void commsReceiverTask() {
    int i;
    vTaskDelay(500);
    char buf[32];
    printToTerminal("COMMS Receiver task started.\n");
    for (;;) {
        uint8_t buf_Rx0[32];
        i = MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));
        if (i) {
            if (buf_Rx0[0] == 0xAA && buf_Rx0[1] == 0xBB && buf_Rx0[2] == 0xCC && xTaskToNotify) {
                xTaskNotify(xTaskToNotify, 0, eNoAction);
            } else {
                /* Ensure null termination */
                if (i >= (int)sizeof(buf_Rx0)) buf_Rx0[sizeof(buf_Rx0)-1] = '\0';
                else buf_Rx0[i] = '\0';

                if (strncmp((char*)buf_Rx0, "SCHED", 5) == 0) {
                    int mm, dd, hh, minu, ssn;
                    if (sscanf((char*)buf_Rx0, "SCHED %d %d %d %d %d", &mm, &dd, &hh, &minu, &ssn) == 5) {
                        timeTaggedTask_t task;
                        task.taskFunction = NULL;
                        /* Preserve current year if available */
                        task.executionTime.year   = 25;
                        task.executionTime.month  = (uint8_t)mm;
                        task.executionTime.day    = (uint8_t)dd;
                        task.executionTime.hour   = (uint8_t)hh;
                        task.executionTime.minute = (uint8_t)minu;
                        task.executionTime.second = (uint8_t)ssn;
                        task.period = 0;
                        task.isRecurring = false;

                        snprintf(buf, sizeof(buf), "Scheduling: %02d/%02d %02d:%02d:%02d\n",
                                 mm, dd, hh, minu, ssn);
                        printToTerminal(buf);

                        scheduleTimeTaggedTask(&task);
                        printToTerminal("OK");

                    } else {
                        printToTerminal("SCHED parse error\n");
                    }
                } else {
                    timeTaggedTask_t task;
                    task.taskFunction = NULL;
                    task.executionTime.second = i;
                    task.period = 0;
                    task.isRecurring = false;
                    scheduleTimeTaggedTask(&task);
                }
                timeTaggedTask_t task;
                task.taskFunction = NULL;
                task.executionTime.second = i;
                task.period = 0;
                task.isRecurring = false;
                scheduleTimeTaggedTask(&task);
                // radioPacket_t packet;
                // packet.len = i;
                // memcpy(packet.data, buf_Rx0, i);
                // packet.type = buf_Rx0[0];
                // xQueueSendToBack(commsRxQueue, &packet, portMAX_DELAY);
            }
        }
        vTaskDelay(5);
    }
}

void sendDataPacket(char *data, int len, uint8_t type) {
    int remLen = len;
    int copyLen = (remLen > 64) ? 64 : remLen;
    int index = 0;

    do {
        radioPacket_t packet;
        packet.header = 0xAA;
        packet.footer = 0xBB;

        packet.len = len;
        packet.index = index;

        index++;
        memcpy(packet.data, data, copyLen);
        remLen -= copyLen;
        packet.type = type;
        uint32_t tempCRC = 69; //crc32b((char*) &packet, 6 + 64);
        packet.crc = tempCRC;

        if (remLen > 64) {
            packet.continued = 1;
            copyLen = 64;
        } else {
            packet.continued = 0;
            copyLen = remLen;
        }

        xQueueSendToBack(commsTxQueue, &packet, portMAX_DELAY);
    } while (remLen > 0);

}

void sendRawData(char* data, int len) {
    // Allocate enough memory for header + data
    radioPacket_t *pkt = pvPortMalloc(sizeof(radioPacket_t) + len);
    if (!pkt) return;

    pkt->header = 0xAA;
    pkt->footer = 0xBB;
    pkt->type = 0x01; // Terminal message type
    pkt->len = len;
    pkt->index = 0;

    memcpy(pkt->data, data, len);  // copies the string bytes

    // **Send pointer to queue** (queue holds telemetryPacket_t*)
    xQueueSendToBack(commsTxQueue, &pkt, portMAX_DELAY);
}

void printToTerminal(char *msg) {
    if(preRtosPrintRaw) {
        custom_MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t*) msg, strlen(msg));
        return;
    }
    
    // Allocate enough memory for header + data
    radioPacket_t *pkt = pvPortMalloc(sizeof(radioPacket_t) + strlen(msg) + 1);
    if (!pkt) return;

    memcpy(pkt->headerStr, pcTaskGetName(NULL), strlen(pcTaskGetName(NULL)));
    pkt->header = 0xAA;
    pkt->footer = 0xBB;
    pkt->type = 0x01; // Terminal message type
    pkt->len = strlen(msg) + 1;
    pkt->index = 0;
    
    memcpy(pkt->data, msg, strlen(msg) + 1);  // copies the string bytes

    // **Send pointer to queue** (queue holds telemetryPacket_t*)
    xQueueSendToBack(commsTxQueue, &pkt, portMAX_DELAY);
}

uint8_t year, month, day, hour, minute, second;
Calendar_t time;
Calendar_t currTime;
char timeBuf[32];



void commsHandlerTask() {
    radioPacket_t packet;
    radioPacket_t rxPacket;

    vTaskDelay(1000);

    printToTerminal("COMMS task started!\n");

    for (;;) {
        if (xQueueReceive(commsRxQueue, &rxPacket, portMAX_DELAY) == pdTRUE) {
            volatile int j = 5;
        }
    }
}
