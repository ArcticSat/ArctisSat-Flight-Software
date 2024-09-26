//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to SPI communication.
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"
#include <firmware/drivers/mss_spi/mss_spi.h> // For the MSS SPI functions

#include "drivers/protocol/spi.h"

#include "drivers/device/adcs_driver.h"


void vTestSPI(void *pvParameters)
{
    uint8_t test_cmd[] = {};
    uint8_t test_wr[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    uint8_t test_rd[7];
//    uint32_t i, useless_number;

    const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000);

//    SPI_enable(&g_mss_spi1);
    for (;;)
    {
//        vTaskSuspendAll();
//        xTaskResumeAll();
    	adcsTxRx(test_wr, 7, NULL, 0);
    	vTaskDelay(pdMS_TO_TICKS(50));
    	adcsTxRx(NULL, 0, test_rd, 7);
    	vTaskDelay(xDelay1000ms);



//        for( i = 0; i < 8; i++)
//		{
//        // Write a block every second.
//        spi_transaction_block_write_without_toggle(
//                    CORE_SPI_1,
//                    SPI_SLAVE_0,
//					&test_wr[i],
//                    1,
//                    NULL,
//                    0
//                );
//		}


//        for( i = 0; i < 2500; i++)
//		{
//			useless_number = 182121*406;
//		}


//        taskYIELD();
//        vTaskSuspendAll();
//        adcsTxRx(NULL, 0, test_rd, 0);

//        memset(test_rd, 0, 4);
//        for( i = 0; i < 0; i++)
//		{
//        spi_transaction_block_read_without_toggle(
//                    CORE_SPI_1,
//                    SPI_SLAVE_0,
//                    NULL,
//                    0,
//                    &test_rx[i],
//                    1
//                );
//		}
//        xTaskResumeAll();
//        vTaskDelay(xDelay1000ms);

//        for( i = 0; i < 250000; i++)
//		{
//			useless_number = 182121*406;
//		}

    }
}
