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

#include "drivers/device/rtc/rtc_ds1393.h"
#include "drivers/mss_rtc/mss_rtc.h"


void vTestRTC(void *pvParameters)
{
    // Test code: reads the external (ds1393) RTC and the internal MSS_RTC directly (low-level only,
    // no mid-level resync/validation helper in this demo build).
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

    for (;;)
    {
        vTaskSuspendAll();
        ds1393_read_time(&buffer);
        MSS_RTC_get_calendar_count(&buffer2);
        xTaskResumeAll();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
