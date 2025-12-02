//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to UART communication.
//
// History
// 2024-07-08 by Mitesh Patel Ft. Aref Asgari
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"
#include "tasks/csp_server.h"

#include <string.h>
#include <csp/csp.h>
#include "csp/interfaces/csp_if_can.h"
#include "drivers/protocol/uart.h"

#include "tasks/telemetry.h"


#include "drivers/filesystem_driver.h"
#define USING_DATA_FLASH
#define USING_PROGRAM_FLASH

//init Params
// 8-bit Gray Scale (RAW, 8-bit for Y only) 03h
// 16-bit Colour (RAW, CrYCbY)  08h
// 16-bit Colour (RAW, 565(RGB))    06h
// JPEG 07h

// param3
//  80 x 60 01h
//  160 x 120   03h
//  128 x 128   09h
//  128 x 96    0Bh

// param4
//  160 x 128   03h
//  320 x 240   05h
//  640 x 480   07h
//  128 x 96    0Bh

lfs_file_t imageFile = { 0 };

int globalFileSize = 0;

struct sPing {
    uint8_t size;
    uint8_t data[16];
};

void vTestUARTTx() {

}
