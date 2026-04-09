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

}

#include <stdint.h>
#include <string.h>

#define CCSDS_VERSION        0
#define CCSDS_TYPE_TM        0
#define CCSDS_SECONDARY_HDR  0   // set to 1 if you want it
#define CCSDS_APID_STRING    69
int build_ccsds_string_packet(uint8_t *out_buf, const char *str)
{
    uint16_t packet_len = strlen(str);  // payload size
    uint16_t ccsds_len  = packet_len + 1; // Length field = (data_bytes - 1)

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

    //16-bit packet type field set to 0x01 for string packets
    out_buf[6] = 0x69;
    out_buf[7] = 0x69;

    // --- PAYLOAD (the string) ---
    memcpy(&out_buf[8], str, packet_len);

    // return total packet size: 6 bytes header + payload
    return 8 + packet_len;
}


static uint16_t seq = 0;

int build_ccsds_data_packet(uint8_t *out_buf, uint16_t type, uint8_t *data, uint8_t data_len)
{
    uint16_t packet_len = data_len;  // payload size
    uint16_t ccsds_len  = packet_len + 1; // Length field = (data_bytes - 1)

    // --- PRIMARY HEADER ---
    uint16_t packet_id;
    // Byte 0-1: Packet ID
    packet_id |= (CCSDS_VERSION & 0x7) << 13;           // version (3 bits)
    packet_id |= (CCSDS_TYPE_TM & 0x1) << 12;           // type    (1 bit)
    packet_id |= (CCSDS_SECONDARY_HDR & 0x1) << 11;     // sec hdr (1 bit)
    packet_id |= (69 & 0x7FF);           // APID    (11 bits)

    out_buf[0] = (packet_id >> 8) & 0xFF;
    out_buf[1] = (packet_id >> 0) & 0xFF;

    // Byte 2-3: Sequence Control
    // Using: packet is standalone (seq flags = 3), count = 0
    //get 14 bits of seq:
    uint16_t seq_count = seq & 0x3FFF;
    uint16_t seq_val = (3 << 14) | seq_count;
    out_buf[2] = (seq_val >> 8) & 0xFF;
    out_buf[3] = (seq_val >> 0) & 0xFF;
    seq++;

    // Byte 4-5: Packet Length
    out_buf[4] = (ccsds_len >> 8) & 0xFF;
    out_buf[5] = (ccsds_len >> 0) & 0xFF;

    //Packet ID
    out_buf[6] = (type >> 8) & 0xFF;
    out_buf[7] = (type >> 0) & 0xFF;

    // --- PAYLOAD (the string) ---
    memcpy(&out_buf[8], data, packet_len);
    // return total packet size: 6 bytes header + payload
    return 8 + packet_len;
}

#define DEBUG_APID 0x69

int generate_manual_cfdp_ack(uint8_t *buf, uint16_t seq_count, uint8_t acked_dir) {
    // Primary Header (6 bytes) + Type (2 bytes)
    buf[0] = 0x18; buf[1] = 0x69; // APID 0x69
    buf[2] = 0xC0 | ((seq_count >> 8) & 0x3F); buf[3] = seq_count & 0xFF;
    buf[4] = 0x00; buf[5] = 0x08; // Length: (2 Type + 7 CFDP) - 1 = 8

    // Custom Type (2 bytes)
    buf[6] = 0xCF; buf[7] = 0xDF;

    // CFDP PDU (7 bytes)
    buf[8]  = 0x20; // Version 1, File Directive
    buf[9]  = 0x03; // ACK body is 3 bytes
    buf[10] = 0x00; // Entity ID len (1 byte)
    buf[11] = 0x01; // Seq len (2 bytes)
    buf[12] = 0x06; // Directive: ACK
    buf[13] = (acked_dir << 4) | 0x01; // Acked Dir + Status Active
    buf[14] = 0x00; // Condition: No Error

    return 15; 
}

int generate_manual_cfdp_nack(uint8_t *buf, uint16_t seq_count, uint32_t start, uint32_t end) {
    // Primary Header (6 bytes) + Type (2 bytes)
    buf[0] = 0x18; buf[1] = 0x69; // APID 0x69
    buf[2] = 0xC0 | ((seq_count >> 8) & 0x3F); buf[3] = seq_count & 0xFF;
    buf[4] = 0x00; buf[5] = 0x0E; // Length: (2 Type + 13 CFDP) - 1 = 14

    // Custom Type (2 bytes)
    buf[6] = 0xCF; buf[7] = 0xDF;

    // CFDP PDU (13 bytes)
    buf[8]  = 0x20; // Version 1, File Directive
    buf[9]  = 0x09; // NACK body is 9 bytes
    buf[10] = 0x00; 
    buf[11] = 0x01;
    buf[12] = 0x08; // Directive: NACK
    
    // Manual Big Endian packing for 'start' offset
    buf[13] = (start >> 24) & 0xFF;
    buf[14] = (start >> 16) & 0xFF;
    buf[15] = (start >> 8) & 0xFF;
    buf[16] = start & 0xFF;

    // Manual Big Endian packing for 'end' offset
    buf[17] = (end >> 24) & 0xFF;
    buf[18] = (end >> 16) & 0xFF;
    buf[19] = (end >> 8) & 0xFF;
    buf[20] = end & 0xFF;

    return 21; // Total packet size
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
      int sendLen = build_ccsds_data_packet(buf, packet->type, (uint8_t*)packet->data, packet->len);
      custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf, sendLen);
      vPortFree(packet);
      xTaskResumeAll();
    }
  }
}

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define CCSDS_PRIMARY_HDR_LEN 6

typedef struct {
    uint16_t apid;
    uint8_t  sec_hdr_flag;
    uint8_t  pkt_type;
    uint16_t seq_count;
    uint16_t data_len;   // number of bytes in data field - 1
    const uint8_t *packet_start;
    size_t total_len;    // header + data
} ccsds_packet_t;

/**
 * Try to extract one CCSDS packet from buffer
 * Returns 1 if a full packet was found, 0 otherwise
 */
int ccsds_extract_packet(const uint8_t *buf, size_t buf_len, ccsds_packet_t *out)
{
    if (buf_len < CCSDS_PRIMARY_HDR_LEN)
        return 0;

    // Primary header
    uint16_t word0 = (buf[0] << 8) | buf[1];
    uint16_t word1 = (buf[2] << 8) | buf[3];
    uint16_t word2 = (buf[4] << 8) | buf[5];

    // Version must be 0 (bits 13..15)
    if ((word0 >> 13) != 0)
        return 0;

    out->pkt_type     = (word0 >> 12) & 0x1;
    out->sec_hdr_flag = (word0 >> 11) & 0x1;
    out->apid         = word0 & 0x07FF;

    out->seq_count    = word1 & 0x3FFF;
    out->data_len     = word2;

    out->total_len = CCSDS_PRIMARY_HDR_LEN + out->data_len + 1;

    if (buf_len < out->total_len)
        return 0;

    out->packet_start = buf;
    return 1;
}

uint8_t buf_Rx0[1024];
ccsds_packet_t extracted_packet;

int create_cfdp_nack_payload(uint8_t *out_data, uint32_t start, uint32_t end) {
    out_data[0] = 0x20; 
    out_data[1] = 0x09; // 1 (code) + 4 (start) + 4 (end)
    out_data[2] = 0x00; 
    out_data[3] = 0x01;
    out_data[4] = 0x08; // Directive: NACK
    
    // Manual Big Endian conversion
    out_data[5] = (start >> 24) & 0xFF; out_data[6] = (start >> 16) & 0xFF;
    out_data[7] = (start >> 8) & 0xFF;  out_data[8] = start & 0xFF;
    out_data[9] = (end >> 24) & 0xFF;   out_data[10] = (end >> 16) & 0xFF;
    out_data[11] = (end >> 8) & 0xFF;  out_data[12] = end & 0xFF;
    return 13;
}

int create_cfdp_ack_payload(uint8_t *out_data, uint8_t acked_dir) {
    out_data[0] = 0x20; // Version 1, File Directive
    out_data[1] = 0x03; // ACK body length (3 bytes)
    out_data[2] = 0x00; // Entity ID len
    out_data[3] = 0x01; // Transaction Seq len
    out_data[4] = 0x06; // Directive: ACK
    out_data[5] = (acked_dir << 4) | 0x01; // EOF + Active Status
    out_data[6] = 0x00; // No Error
    return 7;
}

uint8_t check_if_cfdp_ack_needed(uint8_t *packet) {
    // 1. Check your Type ID at bytes 6-7. 
    // Data uses 0x3400, EOF uses 0x2400 (as seen in your last packet)
    if (packet[6] != 0x34 && packet[6] != 0x24) return 0;

    // 2. CFDP PDU starts at byte 8. 
    // Bit 4 of PDU Byte 0: 1 = Directive, 0 = Data.
    if ((packet[8] & 0x10) == 0x10) {
        // 3. Directive Code is at Packet Byte 12
        uint8_t dir = packet[12];
        // 0x04 (EOF) and 0x05 (Finished) require ACKs
        if (dir == 0x04 || dir == 0x05) return dir;
    }
    return 0;
}

void commsReceiverTask() {
  uint32_t ulNumRecvBytes;
	uint8_t ucUARTByte;
	BaseType_t xResult;
	volatile uint8_t *my_buffer = uart0buffer;
	volatile size_t *uxUnreadBytes = &uxUART0UnreadBytes;

	size_t uxBytesRead;
  vTaskDelay(500);
  printToTerminal("COMMS Receiver task started.\n");
  for (;;) {
    //        i = MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, 32);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (*uxUnreadBytes > 0) {
      taskENTER_CRITICAL();
      {
        memcpy(buf_Rx0, my_buffer, *uxUnreadBytes);
        uxBytesRead = *uxUnreadBytes;
        *uxUnreadBytes -= uxBytesRead;
      }
      taskEXIT_CRITICAL();

      size_t processed = 0;
      while (processed < uxBytesRead) {
        if (ccsds_extract_packet(&buf_Rx0[processed],
                                 uxBytesRead - processed,
                                 &extracted_packet)) {

          uint8_t ack_type = check_if_cfdp_ack_needed(&buf_Rx0[processed]);

          if (ack_type != 0) {
            // 2. Prepare the ACK packet
            // Assuming you have a way to allocate or use a static radioPacket_t
            radioPacket_t* ack_pkt = pvPortMalloc(sizeof(radioPacket_t));
            if (ack_pkt != NULL) {
              ack_pkt->type = 0x2400;  // Your CFDP Type ID for Directives

              // 3. Generate only the 7-byte CFDP PDU
              ack_pkt->len = create_cfdp_ack_payload(ack_pkt->data, ack_type);

              // 4. Send to the TX queue so the satellite replies
              xQueueSendToBack(commsTxQueue, &ack_pkt, 0);
            }
          }
          // Full packet extracted
          static radioPacket_t rxPacket;
          rxPacket.type = extracted_packet.apid;  // using APID as type
          rxPacket.len = extracted_packet.data_len + 1;
          rxPacket.index = extracted_packet.seq_count;
          memcpy(rxPacket.data, &buf_Rx0[processed + CCSDS_PRIMARY_HDR_LEN], rxPacket.len);

          xQueueSendToBack(commsRxQueue, &rxPacket, portMAX_DELAY);

          processed += extracted_packet.total_len;
        } else {
          // No more full packets
          break;
        }
      }
    }
  }
}

void sendDataPacket(char* data, int len, uint8_t type) {

}

void sendRawData(uint8_t* data, uint16_t type, int len) {
  // Allocate enough memory for header + data
  radioPacket_t* pkt = pvPortMalloc(sizeof(radioPacket_t) + len);
  if (!pkt) return;
  pkt->type = type;  // Terminal message type
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
  pkt->type = MSG_TYPE_DEBUG_STRING;  // Terminal message type
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
        if(rxPacket.type == 2045) {
            //file packet
            //log just the data in a file to test
            logADCSTelem((char*)(rxPacket.data), rxPacket.len);
        } else {
            uint8_t CCSDS_command_type = rxPacket.data[9];
            switch (CCSDS_command_type) {
            case 0x05: //dump telemetry command
              telemReadFlag = 1;
              break;
            default:
                break;
            }
          }
    }
  }
}
