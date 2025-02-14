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
    radioPacket_t packet;
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    uint32_t *notVal;
    for (;;) {
        xTaskNotifyWait(0, 0, notVal, portMAX_DELAY);
        if (xQueueReceive(commsTxQueue, &packet, portMAX_DELAY) == pdTRUE) {
            custom_MSS_UART_polled_tx_string(&g_mss_uart0, (char*) &packet,
                    sizeof(radioPacket_t));
        }
    }
}

void commsReceiverTask() {
    int i;
    for (;;) {
        uint8_t buf_Rx0[32];
        i = MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));
        if (i) {
            if (buf_Rx0[0] == 0xAA && buf_Rx0[1] == 0xBB && buf_Rx0[2] == 0xCC
                    && xTaskToNotify) {
                xTaskNotify(xTaskToNotify, 0, eNoAction);
            } else {
                radioPacket_t packet;
                packet.len = i;
                memcpy(packet.data, buf_Rx0, i);
                packet.type = buf_Rx0[0];
                xQueueSendToBack(commsRxQueue, &packet, portMAX_DELAY);
            }
        }
        vTaskDelay(5);
    }
}

void sendDataPacket(char *data, int len, uint8_t type) {
    int remLen = len;
    int copyLen = 64;
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
        uint32_t tempCRC = crc32b((char*) &packet, 6 + 64);
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

void printToTerminal(char *msg) {
    radioPacket_t packet;
    int len = strlen(msg);
    packet.len = len + 1;
    packet.header = 0xAA;
    packet.footer = 0xBB;
    memcpy(packet.data, msg, len);
    packet.type = 0x0A;
    xQueueSendToBack(commsTxQueue, &packet, portMAX_DELAY);
}

void commsHandlerTask() {
    uint8_t buf_Rx0[32];
    uint8_t buf_Rx1[32];

    uint8_t dataBuf[64];

    radioPacket_t packet;
    radioPacket_t rxPacket;

    telemetryPacket_t telemPacket = { 0 };
    telemPacket.data = dataBuf;
    cameraPowerStatus = 0;
    imageFlag = 0;

    for (;;) {
        if (xQueueReceive(commsRxQueue, &rxPacket, portMAX_DELAY) == pdTRUE) {
            int cmd_id = rxPacket.type;
            int size = rxPacket.len;
            switch (cmd_id) {
            case 0x00: //passthrough to power
                printToTerminal("Sending to power");
                sendData(rxPacket.data, size, 2);
                break;
            case 0x01: //passthrough to ADCS
                printToTerminal("Sending to ADCS");
                if (rxPacket.data[1] == 0x11) {
                    cameraPowerStatus = 0;
                }
                if (rxPacket.data[1] == 0x12) {
                    cameraPowerStatus = 1;
                }
                if (rxPacket.data[2] > 32)
                    rxPacket.data[2] = 31;
                adcsArbCommand(rxPacket.data[1], buf_Rx1, rxPacket.data[2]);
                sendDataPacket(buf_Rx1, rxPacket.data[2], 0x10);
                break;
            case 0x02: { //CDH command
                int cdhCommand = rxPacket.data[1];
                switch (cdhCommand) {
                case 0x01:
                    printToTerminal("Take image request received!");
                    takeImage = 1;
                    break;
                case 0x02:
                    printToTerminal("Take image request received!");
                    downlinkImage = 1;
                    break;
                }
                break;
            }
            case 0xAB: //CDH status ping
                dataBuf[0] = powerPingStatus;
                dataBuf[1] = ADCSPingStatus;
                dataBuf[2] = cameraPowerStatus;
                sendDataPacket(dataBuf, 32, 0xAB);
                break;
            case 0xCC: //image status
                if (rxPacket.data[1] == 0xAA) {
                    imageFlag = 0x01;
                    //image ok, send next
                } else {
                    imageFlag = 0xFF;
                    //image bad, resend
                }
                break;
            }
        }
    }
}
