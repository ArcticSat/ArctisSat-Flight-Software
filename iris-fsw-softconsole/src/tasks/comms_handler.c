/*
 * comms_handler.c
 *
 *  Created on: Jan 23, 2025
 *      Author: brend
 */

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include <csp/csp.h>
#include <string.h>

#include "application/adcs.h"
#include "application/application.h"
#include "csp/interfaces/csp_if_can.h"
#include "drivers/protocol/uart.h"
#include "tasks/csp_server.h"
#include "tasks/telemetry.h"
#include "tests.h"
#include "application/CCSDS.h"

#define READ_POINTER_START_ADDR 0x0000
#define WRITE_POINTER_START_ADDR 0x0004

uint32_t crc32b(unsigned char* data, int size) {
  int byteIdx, bitIdx;
  uint32_t crc = 0xFFFFFFFF;
  for (byteIdx = 0; byteIdx < size; byteIdx++) {
    char ch = data[byteIdx];
    for (bitIdx = 0; bitIdx < 8; bitIdx++) {
      uint32_t b = (ch ^ crc) & 1;
      crc >>= 1;
      if (b) crc = crc ^ 0x04C11DB7;
      ch >>= 1;
    }
  }
  return ~crc;
}

volatile int CTS = 1;
static TaskHandle_t xTaskToNotify = NULL;

void sendImagePacket(char* data, int len, int index) {
  int currLen = len;
  radioPacket_t packet;
  packet.header = 0xAA;
  packet.footer = 0xBB;
  packet.index = index;
  packet.len = len;
  memcpy(packet.data, data, len);
  packet.type = 0x99;
  uint32_t tempCRC = crc32b((char*)&packet, 64 + 6);
  packet.crc = tempCRC;
  xQueueSendToBack(commsTxQueue, &packet, portMAX_DELAY);
}


#include <stdint.h>
#include <string.h>

#define CCSDS_VERSION        0
#define CCSDS_TYPE_TM        0
#define CCSDS_SECONDARY_HDR  0   // set to 1 if you want it
#define CCSDS_APID_STRING    10
int build_ccsds_string_packet(uint8_t *out_buf, const char *str)
{
    uint16_t packet_len = strlen(str);  // payload size
    uint16_t ccsds_len  = packet_len - 1; // Length field = (data_bytes - 1)

    // --- PRIMARY HEADER ---

    // Byte 0-1: Packet ID
    uint16_t packet_id = 0;
    packet_id |= (CCSDS_VERSION & 0x7) << 13;           // version (3 bits)
    packet_id |= (CCSDS_TYPE_TM & 0x1) << 12;           // type    (1 bit)
    packet_id |= (CCSDS_SECONDARY_HDR & 0x1) << 11;     // sec hdr (1 bit)
    packet_id |= (CCSDS_APID_STRING & 0x7FF);           // APID    (11 bits)

    out_buf[0] = (packet_id >> 8) & 0xFF;
    out_buf[1] = (packet_id >> 0) & 0xFF;

    // Byte 2-3: Sequence Control
    // Using: packet is standalone (seq flags = 3), count = 0
    uint16_t seq = (3 << 14) | 0;
    out_buf[2] = (seq >> 8) & 0xFF;
    out_buf[3] = (seq >> 0) & 0xFF;

    // Byte 4-5: Packet Length
    out_buf[4] = (ccsds_len >> 8) & 0xFF;
    out_buf[5] = (ccsds_len >> 0) & 0xFF;

    // --- PAYLOAD (the string) ---
    memcpy(&out_buf[6], str, packet_len);

    // return total packet size: 6 bytes header + payload
    return 6 + packet_len;
}

int build_ccsds_data_packet(uint8_t *out_buf, const char *str)
{
    uint16_t packet_len = strlen(str);  // payload size
    uint16_t ccsds_len  = packet_len - 1; // Length field = (data_bytes - 1)

    // --- PRIMARY HEADER ---

    // Byte 0-1: Packet ID
    uint16_t packet_id = 0;
    packet_id |= (CCSDS_VERSION & 0x7) << 13;           // version (3 bits)
    packet_id |= (CCSDS_TYPE_TM & 0x1) << 12;           // type    (1 bit)
    packet_id |= (CCSDS_SECONDARY_HDR & 0x1) << 11;     // sec hdr (1 bit)
    packet_id |= (69 & 0x7FF);           // APID    (11 bits)

    out_buf[0] = (packet_id >> 8) & 0xFF;
    out_buf[1] = (packet_id >> 0) & 0xFF;

    // Byte 2-3: Sequence Control
    // Using: packet is standalone (seq flags = 3), count = 0
    uint16_t seq = (3 << 14) | 0;
    out_buf[2] = (seq >> 8) & 0xFF;
    out_buf[3] = (seq >> 0) & 0xFF;

    // Byte 4-5: Packet Length
    out_buf[4] = (ccsds_len >> 8) & 0xFF;
    out_buf[5] = (ccsds_len >> 0) & 0xFF;

    // --- PAYLOAD (the string) ---
    memcpy(&out_buf[6], str, packet_len);

    // return total packet size: 6 bytes header + payload
    return 6 + packet_len;
}

static uint8_t buf[256];


void commsTransmitterTask() {
  static radioPacket_t* packet;
  xTaskToNotify = xTaskGetCurrentTaskHandle();
  uint32_t* notVal;

  printToTerminal("COMMS Transmitter task started.\n");
  for (;;) {
    // xTaskNotifyWait(0, 0, notVal, portMAX_DELAY);
    if (xQueueReceive(commsTxQueue, &packet, portMAX_DELAY) == pdTRUE) {
      vTaskSuspendAll();
      int sendLen = build_ccsds_string_packet(buf, (char*)packet->data);
      custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf,
                                       sendLen);
      sendLen = build_ccsds_data_packet(buf, (char*)packet->data);
      custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf,
                                       sendLen);
      vPortFree(packet);
      xTaskResumeAll();
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
    // read until you get a newline character
    while (MSS_UART_get_rx(&g_mss_uart0, &buf_Rx0[idx], 1)) {
      if (buf_Rx0[idx] == '\n' || idx >= 31) {
        buf_Rx0[idx] = '\0';
        break;
      }
      idx++;
    }

    vTaskDelay(5);
  }
}

void sendDataPacket(char* data, int len, uint8_t type) {
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
    uint32_t tempCRC = 69;  // crc32b((char*) &packet, 6 + 64);
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
  radioPacket_t* pkt = pvPortMalloc(sizeof(radioPacket_t) + len);
  if (!pkt) return;

  pkt->header = 0xAA;
  pkt->footer = 0xBB;
  pkt->type = 0x01;  // Terminal message type
  pkt->len = len;
  pkt->index = 0;

  memcpy(pkt->data, data, len);  // copies the string bytes

  // **Send pointer to queue** (queue holds telemetryPacket_t*)
  xQueueSendToBack(commsTxQueue, &pkt, portMAX_DELAY);
}

void printToTerminal(char* msg) {
  if (preRtosPrintRaw) {
    custom_MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t*)msg,
                                     strlen(msg));
    return;
  }

  // Allocate enough memory for header + data
  radioPacket_t* pkt = pvPortMalloc(sizeof(radioPacket_t) + strlen(msg) + 1);
  if (!pkt) return;

  memcpy(pkt->headerStr, pcTaskGetName(NULL), strlen(pcTaskGetName(NULL)));
  pkt->header = 0xAA;
  pkt->footer = 0xBB;
  pkt->type = 0x01;  // Terminal message type
  pkt->len = strlen(msg) + 1;
  pkt->index = 0;

  memcpy(pkt->data, msg, strlen(msg) + 1);  // copies the string bytes

  // **Send pointer to queue** (queue holds telemetryPacket_t*)
  xQueueSendToBack(commsTxQueue, &pkt, 0);
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
