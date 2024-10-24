//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to the real time clock.
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"

#include "drivers/device/rtc/rtc_time.h"


void vTestRTC(void *pvParameters)
{
    // Test code
    static volatile int error_occurred = 0;

    static Calendar_t buffer = {
            00, // seconds
            32, // minutes
            15, // hours
            9, // day
            8, // August
            24, // year (2024)
            5, // weekday (Thursday)
            1, // week (not used), HOWEVER it must be 1 or greater.
    };
//    uint8_t buf_readreg[1];

    static Calendar_t buffer2;

    vTaskSuspendAll();
//    ds1393_write_reg(0x8d, 0x38);
//    ds1393_read_reg(0x0d, buf_readreg);

//    ds1393_write_time(&buffer);
    if (TIME_SUCCESS != resync_rtc())
    {
        error_occurred = 1;
    }
    xTaskResumeAll();

    for (;;)
    {
        vTaskSuspendAll();
        ds1393_read_time(&buffer);
        read_rtc(&buffer2);
        xTaskResumeAll();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
