/*
 * comms_handler.c
 *
 *  Created on: Jan 23, 2025
 *      Author: brend
 *
 * NOTE (demo simplification): this file used to contain a full production
 * ground-comms protocol stack (CCSDS framing + a hand-rolled CFDP ACK/NACK
 * layer) on top of UART. For this demo build that has been stripped out.
 * What remains is the minimal raw-UART pipe that backs printToTerminal(),
 * which is used throughout the whole demo for logging.
 *
 * The actual "CSP transmit/receive demo" pair lives elsewhere:
 *  - CSPClient() in main.c reads txQueue and transmits over CSP.
 *  - vCSP_Server() in tasks/csp_server.c accepts CSP connections, reads
 *    packets, and logs them via printToTerminal().
 */

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "drivers/protocol/uart.h"
#include "tasks/comms_handler.h"
#include "tasks/telemetry.h"

static uint8_t rxScratchBuf[256];

// Sends a queued radioPacket_t's raw payload out over UART.
void commsTransmitterTask() {
  static radioPacket_t* packet;

  printToTerminal("COMMS Transmitter task started.\n");
  for (;;) {
    if (xQueueReceive(commsTxQueue, &packet, portMAX_DELAY) == pdTRUE) {
      vTaskSuspendAll();
      custom_MSS_UART_polled_tx_string(&g_mss_uart0, packet->data, packet->len);
      vPortFree(packet);
      xTaskResumeAll();
    }
  }
}

// Reads raw bytes off the UART Rx buffer (filled by the UART ISR) and
// forwards them, unparsed, to commsHandlerTask via commsRxQueue.
void commsReceiverTask() {
  volatile uint8_t* my_buffer = uart0buffer;
  volatile size_t* uxUnreadBytes = &uxUART0UnreadBytes;
  size_t uxBytesRead;

  vTaskDelay(500);
  printToTerminal("COMMS Receiver task started.\n");
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (*uxUnreadBytes > 0) {
      // Critical section to safely read from the UART buffer and update the
      // unread byte count (unread byte count is updated by the UART ISR when
      // new data arrives).
      taskENTER_CRITICAL();
      {
        uxBytesRead = *uxUnreadBytes;
        if (uxBytesRead > sizeof(rxScratchBuf)) {
          uxBytesRead = sizeof(rxScratchBuf);
        }
        memcpy(rxScratchBuf, (const void*)my_buffer, uxBytesRead);
        *uxUnreadBytes -= uxBytesRead;
      }
      taskEXIT_CRITICAL();

      static radioPacket_t rxPacket;
      rxPacket.type = MSG_TYPE_DEBUG_STRING;
      rxPacket.len = (uint8_t)((uxBytesRead > sizeof(rxPacket.data)) ? sizeof(rxPacket.data) : uxBytesRead);
      rxPacket.index = 0;
      memcpy(rxPacket.data, rxScratchBuf, rxPacket.len);

      xQueueSendToBack(commsRxQueue, &rxPacket, portMAX_DELAY);
    }
  }
}

// Queues a raw block of data to be sent out over UART as a radioPacket_t.
void sendRawData(uint8_t* data, uint16_t type, int len) {
  radioPacket_t* pkt = pvPortMalloc(sizeof(radioPacket_t) + len);
  if (!pkt) return;
  pkt->type = type;
  pkt->len = len;
  pkt->index = 0;

  memcpy(pkt->data, data, len);

  xQueueSendToBack(commsTxQueue, &pkt, portMAX_DELAY);
}

// The main terminal logging entry point used throughout the demo.
void printToTerminal(char* msg) {
  if (preRtosPrintRaw) {
    custom_MSS_UART_polled_tx_string(&g_mss_uart0, (const uint8_t*)msg, strlen(msg));
    return;
  }

  radioPacket_t* pkt = pvPortMalloc(sizeof(radioPacket_t) + strlen(msg) + 1);
  if (!pkt) return;

  memcpy(pkt->headerStr, pcTaskGetName(NULL), strlen(pcTaskGetName(NULL)));
  pkt->type = MSG_TYPE_DEBUG_STRING;
  pkt->len = strlen(msg) + 1;
  pkt->index = 0;

  memcpy(pkt->data, msg, strlen(msg) + 1);

  xQueueSendToBack(commsTxQueue, &pkt, 0);
}

// Demo consumer of incoming UART data: just logs what came in.
void commsHandlerTask() {
  radioPacket_t rxPacket;

  vTaskDelay(1000);

  printToTerminal("COMMS task started!\n");

  for (;;) {
    if (xQueueReceive(commsRxQueue, &rxPacket, portMAX_DELAY) == pdTRUE) {
      char logbuf[96];
      int printable = (rxPacket.len < sizeof(rxPacket.data)) ? rxPacket.len : (int)sizeof(rxPacket.data) - 1;
      if (printable > (int)sizeof(logbuf) - 40) {
        printable = (int)sizeof(logbuf) - 40;
      }
      snprintf(logbuf, sizeof(logbuf), "COMMS RX (%u bytes): %.*s\n", (unsigned)rxPacket.len, printable, rxPacket.data);
      printToTerminal(logbuf);
    }
  }
}
