//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to UART communication.
//
// History
// 2024-07-08 by Mitesh Patel
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"

#include <string.h>
#include <csp/csp.h>
#include "csp/interfaces/csp_if_can.h"
#include "drivers/protocol/uart.h"


void vTestUARTTx()
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

//    const char message[] = "Test";
//    memcpy(buf_Tx, message, sizeof(message));

    vTaskDelay(2000);


    for (;;)
    {
    	Tx_Success = 0;

//    	prvUARTSend(&g_mss_uart0, buf_Tx, sizeof(buf_Tx));
//    	Tx_Success = MSS_UART_tx_complete(&g_mss_uart0);
//        vTaskDelay(delay);

//    	MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx);
    	i = MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));
    	if(i > 0) {
    	    custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));
    	    csp_conn_t * conn;
            csp_packet_t * packet;
            conn = csp_connect(2,0,2,1000,0);   //Create a connection. This tells CSP where to send the data (address and destination port).
            packet = csp_buffer_get(sizeof(i)); // Get a buffer large enough to fit our data. Max size is 256.
            sprintf(packet->data, buf_Rx0);
            packet->length=i;

            csp_send(conn,packet,0);
            csp_close(conn);
            csp_buffer_free(packet);
            vTaskDelay(100);
    	}


//    	for( i = 0; i < 2500000; i++)
//		{
//			useless_number = 182121*406;
//		}
//
//    	custom_MSS_UART_polled_tx_string(&g_mss_uart1, buf_Tx1, sizeof(buf_Tx1));
//		for( i = 0; i < 2500000; i++)
//		{
//			useless_number = 182121*406;
//		}
//		MSS_UART_get_rx(&g_mss_uart1, buf_Rx1, sizeof(buf_Rx1));
        vTaskDelay(100);


    }
}



