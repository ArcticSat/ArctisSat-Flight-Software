//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to the watchdog timer.
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"
#include "drivers/mss_gpio/mss_gpio.h"
#include "drivers/device/watchdog.h"
#include "drivers/mss_gpio/mss_gpio.h"
#include "tasks/telemetry.h"
#include "application/memory_manager.h"

static int last_reboot_wd = 0;
static int external_wd_state = 1;
static int external_wd_state_prev = 1;

void start_stop_external_wd(uint8_t startStop) {

    external_wd_state = startStop;
}

void vTestWD(void *pvParameters) {
    // In the future, this task could be used as a reset service. For instance, tasks could:
    // - Check-in to this task. If a task fails to check-in as expected, the watchdog would be left to reset.
    // - Request a reset.

    // Note that the watchdog is not enabled (by the MSS) for certain situations, such as:
    // - While debugging.
    // - Programming.

    //Dont print from this task! Current stack size is too small

    if (timeout_occured_WD()) {
        clear_timeout_WD();
        last_reboot_wd = 1;

        // TODO - Log event!
    } else {
        // TODO - Log event!
    }
    uint8_t pinState = 0;

//    uint8_t pinState=0;

    //Set the WDSEL to VCC -> 1.6s timeout
    MSS_GPIO_set_output(MSS_GPIO_19, 1);

    int started = 0;
    for (;;) {
            MSS_GPIO_set_output(MSS_GPIO_18, pinState);
            pinState = ~pinState;
            service_WD();
            vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
