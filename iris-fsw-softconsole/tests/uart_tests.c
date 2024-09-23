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

#include "drivers/protocol/uart.h"


void vTestUARTTx()
{
    const TickType_t delay = pdMS_TO_TICKS(2000);

    const uint8_t buf_Tx[] = {0xc0, 0x0e, 0x11, 0xa1, 0x00, 0x20, 0x00, 0x00, 0x8e, 0x51, 0xc0};
//    const uint8_t buf_Tx[] = {0xc0, 0x0e, 0x11, 0xa0, 0x6b, 0x00, 0xc0};
    int8_t Tx_Success = 0;
    uint32_t i, useless_number;

    uint8_t buf_Rx[32];

//    const char message[] = "Test";
//    memcpy(buf_Tx, message, sizeof(message));

    for (;;)
    {
    	Tx_Success = 0;

//    	prvUARTSend(&g_mss_uart0, buf_Tx, sizeof(buf_Tx));
//    	Tx_Success = MSS_UART_tx_complete(&g_mss_uart0);
//        vTaskDelay(delay);

//    	MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx);
    	custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx, sizeof(buf_Tx));

    	for( i = 0; i < 2500000; i++)
    	{
    		useless_number = 182121*406;
    	}

    	MSS_UART_get_rx(&g_mss_uart0, buf_Rx, sizeof(buf_Rx));


    }
}




