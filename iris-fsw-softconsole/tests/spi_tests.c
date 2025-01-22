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


//void vTestSPI(void *pvParameters)
//{
//    uint8_t test_cmd[] = {};
//    uint8_t test_wr[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
//    uint8_t test_rd[7];
////    uint32_t i, useless_number;
//
//    const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000);
//
////    SPI_enable(&g_mss_spi1);
//    for (;;)
//    {
////        vTaskSuspendAll();
////        xTaskResumeAll();
//    	adcsTxRx(test_wr, 7, NULL, 0);
//    	vTaskDelay(pdMS_TO_TICKS(50));
//    	adcsTxRx(NULL, 0, test_rd, 7);
//    	vTaskDelay(xDelay1000ms);
//
//
//
////        for( i = 0; i < 8; i++)
////		{
////        // Write a block every second.
////        spi_transaction_block_write_without_toggle(
////                    CORE_SPI_1,
////                    SPI_SLAVE_0,
////					&test_wr[i],
////                    1,
////                    NULL,
////                    0
////                );
////		}
//
//
////        for( i = 0; i < 2500; i++)
////		{
////			useless_number = 182121*406;
////		}
//
//
////        taskYIELD();
////        vTaskSuspendAll();
////        adcsTxRx(NULL, 0, test_rd, 0);
//
////        memset(test_rd, 0, 4);
////        for( i = 0; i < 0; i++)
////		{
////        spi_transaction_block_read_without_toggle(
////                    CORE_SPI_1,
////                    SPI_SLAVE_0,
////                    NULL,
////                    0,
////                    &test_rx[i],
////                    1
////                );
////		}
////        xTaskResumeAll();
////        vTaskDelay(xDelay1000ms);
//
////        for( i = 0; i < 250000; i++)
////		{
////			useless_number = 182121*406;
////		}
//
//    }
//}



void vTestSPI(void *pvParameters){
    const uint8_t buf_Tx0[] = {66};
    const uint8_t buf_Tx1[] = {105};
    uint8_t adcs_status = 0;
    const uint8_t cmdID_42 = 66;
	const uint8_t cmdID_69 = 105;
	uint8_t cmdID = 0;
    uint8_t buf_Rx0[32];
    buf_Rx0[0] = 0x50;
    uint8_t counter = 0;
    for (;;)
    {
//    	buf_Rx0[0] = 0x00;
    	uint8_t test_rd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    	adcs_status = 1;
    	if (counter == 0){
    		cmdID = 0x69;
    		counter++;
    	}
    	else if (counter == 1){
    		cmdID = 0x42;
    		counter++;
    	}
    	else if (counter == 2){
    		cmdID = 0x15;
			counter++;
		}
    	else if (counter == 3){
    		cmdID = 0x18;
			counter++;
		}
    	else if (counter == 4){
    		cmdID = 0x99;
			counter = 0;
		}
		adcs_status = adcsSyncSpiCommand(cmdID);
		vTaskDelay(pdMS_TO_TICKS(50));
		if (adcs_status == 0){
			adcsTxRx(NULL, 0, test_rd, sizeof(test_rd));
		}
    	vTaskDelay(2000);
    }
}
