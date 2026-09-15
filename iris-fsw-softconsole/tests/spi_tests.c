//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to SPI communication.
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
// 2026-09-15
// - Simplified for the demo build: this used to drive the ADCS SPI protocol
//   (adcsSyncSpiCommand()/adcsTxRx()), which has been removed. Replaced with a
//   generic SPI core exerciser that isn't tied to any particular device.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"
#include <firmware/drivers/mss_spi/mss_spi.h> // For the MSS SPI functions

#include "drivers/protocol/spi.h"

// Generic SPI exerciser. Sends a single command byte and reads back a
// response on CORE_SPI_1 / SPI_SLAVE_0. This isn't tied to any particular
// device, it's just meant to demonstrate that the SPI mid-level driver works.
void vTestSPI(void *pvParameters){
    const uint8_t cmd[] = {0x9F}; // Placeholder "read ID"-style command byte.
    uint8_t rd_buf[4] = {0};

    for (;;)
    {
        spi_transaction_block_read_without_toggle(CORE_SPI_1, SPI_SLAVE_0, (uint8_t*)cmd, sizeof(cmd), rd_buf, sizeof(rd_buf));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
