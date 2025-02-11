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

uint32_t crc32b(unsigned char *message) {
   int i, j;
   unsigned int byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   for(int i = 0; i < 68; i++){
      byte = message[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}

void sendImagePacket(char* data, int len, int index) {
    int currLen = len;
    radioPacket_t packet;
    packet.header = 0xAA;
    packet.footer = 0xBB;
    packet.index = index;
    packet.len = len;
    memcpy(packet.data, data, len);
    packet.type = 0x99;
    uint32_t tempCRC = crc32b((char*) &packet);
    packet.crc = tempCRC;
    xQueueSendToBack(commsQueue, &packet, 0);
}

void sendDataPacket(char* data, int len, uint8_t type) {
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
        uint32_t tempCRC = crc32b((char*) &packet);
        packet.crc = tempCRC;
        if(remLen > 64) {
            packet.continued = 1;
            copyLen = 64;
        } else {
            packet.continued = 0;
            copyLen = remLen;
        }
        xQueueSendToBack(commsQueue, &packet, 0);
    } while(remLen > 0);

}

void printToTerminal(char* msg) {
    radioPacket_t packet;
    int len = strlen(msg);
    packet.len = len + 1;
    packet.header = 0xAA;
    packet.footer = 0xBB;
    memcpy(packet.data, msg, len);
    packet.type = 0x0A;
    xQueueSendToBack(commsQueue, &packet, 0);
}

void commsHandlerTask()
{
    const TickType_t delay = pdMS_TO_TICKS(2000);

//    const uint8_t buf_Tx0[] = {0xc0, 0x0e, 0x11, 0xa1, 0x00, 0x20, 0x00, 0x00, 0x8e, 0x51, 0xc0};
//    const uint8_t buf_Tx1[] = {0xc0, 0x0e, 0x11, 0xa1, 0x00, 0x20, 0x00, 0x00, 0x8e, 0x51, 0xc0};

    const uint8_t buf_Tx0[] = {0x01, 0x02, 0x03};

    const uint8_t buf_Tx1[] = {0x11, 0x12, 0x13, 0x00};

    int8_t Tx_Success = 0;
    uint32_t i, useless_number;

    uint8_t buf_Rx0[32];
    uint8_t buf_Rx1[32];

    uint8_t dataBuf[64];

    radioPacket_t packet;
    telemetryPacket_t telemPacket = {0};
    telemPacket.data = dataBuf;

    imageFlag = 0;

//    const char message[] = "Test";
//    memcpy(buf_Tx, message, sizeof(message));

    vTaskDelay(2000);


    for (;;)
    {
        Tx_Success = 0;

//      prvUARTSend(&g_mss_uart0, buf_Tx, sizeof(buf_Tx));
//      Tx_Success = MSS_UART_tx_complete(&g_mss_uart0);
//        vTaskDelay(delay);

//      MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx);
        i = MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));
        if(i > 0) {
            int cmd_id = buf_Rx0[0];
            switch(cmd_id){
                case 0x00: //passthrough to power
                    printToTerminal("Sending to power");
                    sendData(buf_Rx0, i, 2);
                    break;
                case 0x01: //passthrough to ADCS
                    printToTerminal("Sending to ADCS");
                    telemPacket.telem_id = buf_Rx0[1];
                    adcsArbCommand(buf_Rx0[1], &buf_Rx1, buf_Rx0[2]);
                    sendDataPacket(&buf_Rx1, buf_Rx0[2], 0x10);
                    break;
                case 0xAB: //CDH status ping
                    dataBuf[0] = powerPingStatus;
                    dataBuf[1] = ADCSPingStatus;
                    sendDataPacket(dataBuf, 32, 0xAB);
                    break;
                case 0xCC: //image status
                    if(buf_Rx0[1] == 0xAA) {
                        imageFlag = 0x01;
                        //image ok, send next
                    } else {
                        imageFlag = 0xFF;
                        //image bad, resend
                    }
                    break;
            }

//          custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));
//          csp_conn_t * conn;
//            csp_packet_t * packet;
//            conn = csp_connect(2,0,2,1000,0);   //Create a connection. This tells CSP where to send the data (address and destination port).
//            packet = csp_buffer_get(sizeof(i)); // Get a buffer large enough to fit our data. Max size is 256.
//            sprintf(packet->data, buf_Rx0);
//            packet->length=i;
//
//            csp_send(conn,packet,0);
//            csp_close(conn);
//            csp_buffer_free(packet);
//            vTaskDelay(100);
        }

        if(xQueueReceive(commsQueue, &packet, 0) == pdTRUE) {
            custom_MSS_UART_polled_tx_string(&g_mss_uart0, (char*) &packet, sizeof(radioPacket_t));
        }


//      for( i = 0; i < 2500000; i++)
//      {
//          useless_number = 182121*406;
//      }
//
//      custom_MSS_UART_polled_tx_string(&g_mss_uart1, buf_Tx1, sizeof(buf_Tx1));
//      for( i = 0; i < 2500000; i++)
//      {
//          useless_number = 182121*406;
//      }
//      MSS_UART_get_rx(&g_mss_uart1, buf_Rx1, sizeof(buf_Rx1));
        vTaskDelay(100);


    }
}
