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

#define READ_POINTER_START_ADDR 0x0000
#define WRITE_POINTER_START_ADDR 0x0004

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
    static radioPacket_t *packet;
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
//        i = MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, 32);
        
        uint8_t idx = 0;
        //read until you get a newline character
        while(MSS_UART_get_rx(&g_mss_uart0, &buf_Rx0[idx], 1)) {
            if(buf_Rx0[idx] == '\n' || idx >= 31) {
                buf_Rx0[idx] = '\0';
                break;
            }
            idx++;
        }


        if (idx > 0) {
            if (buf_Rx0[0] == 0xAA && buf_Rx0[1] == 0xBB && buf_Rx0[2] == 0xCC && xTaskToNotify) {
                xTaskNotify(xTaskToNotify, 0, eNoAction);
            } else {
                /* Ensure null termination */
                if (i >= (int)sizeof(buf_Rx0)) buf_Rx0[sizeof(buf_Rx0)-1] = '\0';
                else buf_Rx0[i] = '\0';

                if (strncmp((char*)buf_Rx0, "SHD", 3) == 0) {
                    int mm, dd, hh, minu, ssn;
                    if (sscanf((char*)buf_Rx0, "SHD %d %d %d %d %d", &mm, &dd, &hh, &minu, &ssn) == 5) {
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
                        printToTerminal("SHD parse error\n");
                    }
                } else {
                    //i am going to have another test command for setting the time
                    //it will be TIME HH MM SS please do that
                    if (strncmp((char*)buf_Rx0, "TIME", 4) == 0) {
                        int hh, mm, ss;
                        if (sscanf((char*)buf_Rx0, "TIME %d %d %d", &hh, &mm, &ss) == 3) {
                            Calendar_t currTime;
                            currTime.hour = (uint8_t)hh;
                            currTime.minute = (uint8_t)mm;
                            currTime.second = (uint8_t)ss;
                            set_rtc(&currTime);

                            snprintf(buf, sizeof(buf), "RTC Time Set to: %02d:%02d:%02d\n",
                                     hh, mm, ss);
                            printToTerminal(buf);
                            ds1393_read_time(&currTime);
                            snprintf(buf, sizeof(buf), "RTC Time Now: %02d:%02d:%02d\n",
                                     currTime.hour, currTime.minute, currTime.second);
                            printToTerminal(buf);
                        } else {
                            printToTerminal("TIME parse error\n");
                        }
                        //next add the DATE command that takes DATE YY MM DD
                    } else if (strncmp((char*)buf_Rx0, "DATE", 4) == 0) {
                        int yy, mm, dd;
                        if (sscanf((char*)buf_Rx0, "DATE %d %d %d", &yy, &mm, &dd) == 3) {
                            Calendar_t currDate;
                            ds1393_read_time(&currDate);
                            currDate.year = (uint8_t)yy;
                            currDate.month = (uint8_t)mm;
                            currDate.day = (uint8_t)dd;
                            set_rtc(&currDate);
                        } else {
                            printToTerminal("DATE parse error\n");
                        }
                    } else if (strncmp((char*)buf_Rx0, "DUMPTEL", 7) == 0) {
                        telemReadFlag = 1;
                        printToTerminal("Telemetry dump flag set.\n");
                    }
                }
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
        // asMram_write(0x00, msg, strlen(msg));
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
