//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Main entry point for the project. Created using the demo project given as a starting point:
//    - https://www.digikey.com/eewiki/display/microcontroller/Getting+Started+with+the+Microsemi+SmartFusion+2+Maker-Board
//
// History
// 2019-01-16 by Tamkin Rahman and Joseph Howarth
// - Removed UART1 and IoT node code.
// 2019-02-08 by Tamkin Rahman
// - Add test code for SPI.
// 2019-02-24 by Tamkin Rahman
// - Update test code for SPI, and create a second task for SPI tests.
// 2019-03-28 by Tamkin Rahman
// - Add test code for CAN.
// 2019-04-16 by Tamkin Rahman
// - Add test code for watchdog and rtc.
// 2019-06-23 by Tamkin Rahman
// - Add test code for MRAM
// - Update test code for RTC to remove traps.
// - Prevent task switching instead of using mutexes for SPI read/write.
// 2019-06-09 by Joseph Howarth
// - Add test code for flash.
// 2020-01-03 by Joseph Howarth
// - Add test code for ADCS driver.

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
/******************************************************************************
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * configCREATE_SIMPLE_BLINKY_DEMO_ONLY setting (defined in FreeRTOSConfig.h) is
 * used to select between the two.  The simply blinky demo is implemented and
 * described in main_blinky.c.  The more comprehensive test and demo application
 * is implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and FreeRTOS hook functions.
 *
 */

/* Standard includes. */

/* Library includes */
#include <csp/csp.h>
#include "csp/interfaces/csp_if_can.h"

/* Kernel includes. */
#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/queue.h>
#include <FreeRTOS-Kernel/include/semphr.h>
#include <FreeRTOS-Kernel/include/task.h>

/* Driver includes. */
#include "drivers/mss_uart/mss_uart.h"    // For baud rate defines and instances

/* Application includes. */
#include "main.h"
#include "taskhandles.h"
//#include "drivers/protocol/can.h"
#include "drivers/device/memory/flash_common.h"
#include "drivers/device/leds.h"
#include "drivers/device/memory/mram.h"
#include "drivers/device/rtc/rtc_time.h"
#include "drivers/protocol/spi.h"
#include "drivers/protocol/uart.h"
#include "drivers/device/watchdog.h"
#include "tasks/scheduler.h"
#include "tasks/priority_queue.h"
#include "drivers/device/adcs_driver.h"
#include "drivers/filesystem_driver.h"
#include "tests.h"
#include "tasks/telemetry.h"
#include "tasks/csp_server.h"
#include "tasks/fw_update_mgr.h"
#include "drivers/device/adc/AD7928.h"
#include "application/cdh.h"
//#include "application/detumbling.h"
#include "application/sun_pointing.h"
#include "application/memory_manager.h"
#include "tasks/comms_handler.h"
#include "application/application.h"

#include "tasks/healthAndSafety.h"

//#define SERVER
//#define CLIENT
//#define CAN_SERVER
#define CSP_SERVER

TaskHandle_t xUART0RxTaskToNotify;
TaskHandle_t vTTTScheduler_h;
TaskHandle_t vFw_Update_Mgr_Task_h;
TaskHandle_t vCanServer_h;
TaskHandle_t vCSP_Server_h;
TaskHandle_t vTestWD_h;
TaskHandle_t vSunPointing_h;
TaskHandle_t vTestAdcsDriverInterface_h;

HardwareCheck_t setupHardwareStatus = { 0 };

static void prvSetupHardware(void);
static void vTestCspServer(void *pvParameters);
static void vTestCspClient(void *pvParameters);
static void vTestingTask(void *pvParams);

void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

static void delay_cycles(volatile uint32_t incycles)
{
    uint32_t cycles = incycles * 3;
    while (cycles--) {
        __asm volatile("nop");
    }
}

int main(void) {
    preRtosPrintRaw = 1;
    prvSetupHardware();

    BaseType_t status;

    printToTerminal("\n\n###########  Queue init...  ############\n");

    txQueue = xQueueCreate(5, sizeof(satPacket));
    commsTxQueue = xQueueCreate(10, sizeof(radioPacket_t*));
    commsRxQueue = xQueueCreate(10, sizeof(radioPacket_t));
    telemetryQueue = xQueueCreate(5, sizeof(mytelemetryPacket_t*));
    taskQueue = xQueueCreate(3, sizeof(timeTaggedTask_t));
    errorQueue = xQueueCreate(5, sizeof(caughtError_t));

    printToTerminal("Queue init done.\n");

    logMessage("System booting...\n");


    //    status = xTaskCreate(vTestFlashFull,"Test Flash",6000,(void *)flash_devices[DATA_FLASH],1,NULL);
    //	  status = xTaskCreate(vTestSPI,"Test SPI",1000,NULL,10,NULL);
    //	  status = xTaskCreate(vTestFlash,"Test Flash",2000,(void *)flash_devices[DATA_FLASH],1,NULL);
    //	  status = xTaskCreate(vTestFlashFull,"Test Flash",2000,(void *)flash_devices[PROGRAM_FLASH],1,NULL);
//        status = xTaskCreate(vTestMRAM,"Test MRAM",512,NULL,1,NULL);
    //    status = xTaskCreate(vTestRTC,"Test RTC",configMINIMAL_STACK_SIZE,NULL,1,NULL);
    //    status = xTaskCreate(vTestSPI,"Test SPI",1000,NULL,1,NULL);
    //    status = xTaskCreate(vTestSPI,"Test SPI2",1000,NULL,1,NULL);
    //    status = xTaskCreate(vTestCANTx,"Test CAN Tx",configMINIMAL_STACK_SIZE,NULL,1,NULL);
    //    status = xTaskCreate(vTestCANRx,"Test CAN Rx",500,NULL,10,NULL);
    //    status = xTaskCreate(vTestCspServer,"Test CSP Server",1000,NULL,1,NULL);
    //    status = xTaskCreate(vTestCanServer,"Test CAN Rx",1000,NULL,2,&vTestCanServer_h);
    //    status = xTaskCreate(vTaskTest_Priority_Queue,"Test Priority_Queue",256,NULL,1,NULL);
    //    status = xTaskCreate(vTestTaskScheduler,"Test time tagged task queue",256,NULL,1,NULL);
    //    status = xTaskCreate(vTestADC, "adcTest", 160, NULL, 1, NULL);
    //    status = xTaskCreate(vCanServer,"CAN Rx",1000,NULL,2,&vCanServer_h);
    //    status = xTaskCreate(vTestAdcsDriver, "ADCS handler", 500, NULL, 1, NULL);

    /**THESE ARE THE MAIN FUNCTIONS**/

    printToTerminal("\n\n###########  Begin task init...  ############\n");


    // TODO add handling and logging to all the task status
    // Need to suspend this task until the CSP stack is up and running.
    // If we start pumping CAN messages into CSP before it is ready it will crash.
    // The task is resumed in the CSP server once it is ready.
//    status = xTaskCreate(vCanServer, "CAN Rx", 300, NULL, 1, &vCanServer_h);
//    vTaskSuspend(vCanServer_h);
    // This task handles all incoming CSP packets and routes them to the appropriate handler.
    status = xTaskCreate(vCSP_Server, "cspServer ", 200, NULL, 1,
            &vCSP_Server_h);
    printToTerminal("CSP Server task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    volatile int j = xPortGetFreeHeapSize();


    // This task reads from the tx queue and sends packets out over CSP.
    // TODO rename this task
    status = xTaskCreate(vTestCspClient, "CSP Tx    ", 200, NULL, 1, NULL);
    printToTerminal("CSP Client task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    j = xPortGetFreeHeapSize();


    // This task reads from the UART Rx queue and handles them
    status = xTaskCreate(commsHandlerTask, "UARTHandle", 300, NULL, 1, NULL);
    printToTerminal("UART Handler task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    j = xPortGetFreeHeapSize();


    // This task dispatches packets in the TX queue over UART
    status = xTaskCreate(commsTransmitterTask, "UART Tx   ", 200, NULL, 1, NULL);
    printToTerminal("UART Transmitter task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");    
    j = xPortGetFreeHeapSize();


    // This task reads from the UART and puts packets in the Rx queue
    status = xTaskCreate(commsReceiverTask, "UART Rx   ", 300, NULL, 1, NULL);
    printToTerminal("UART Receiver task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    j = xPortGetFreeHeapSize();


//    // This task drives ADCS
//    status = xTaskCreate(vADCSDriver, "ADCS handler", 100, NULL, 1, NULL);
//    printToTerminal("ADCS Driver task created. Status: ");
//    delay_cycles(100000);
//    printToTerminal(status ? "Success\n" : "Failure\n");
//    j = xPortGetFreeHeapSize();


    // This task drives power
    status = xTaskCreate(vPowerDriver, "PWRHandler", 200, NULL, 1, NULL);
    printToTerminal("Power Driver task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    j = xPortGetFreeHeapSize();


    // TODO fix this task
    status = xTaskCreate(telemetryManager, "Telemetry ", 500, NULL, 1, NULL);
    printToTerminal("Telemetry Manager task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    j = xPortGetFreeHeapSize();


    //Main loop, does all the typical stuff
    status = xTaskCreate(vMissionLoop, "Mission   ", 500, NULL, 1, NULL);
    printToTerminal("Mission Operations Loop task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    j = xPortGetFreeHeapSize();


    status = xTaskCreate(vHealthAndSafety, "HlthSafety", 700, NULL, 1,
            NULL);
    printToTerminal("Health and Safety task created. Status: ");
    delay_cycles(100000);
    printToTerminal(status ? "Success\n" : "Failure\n");
    j = xPortGetFreeHeapSize();


    printToTerminal("Free heap bytes before scheduler start: ");
    char heapStr[20];
    snprintf(heapStr, 20, "%lu\n", j);
    printToTerminal(heapStr);
    printToTerminal("\n\n###########  Starting scheduler...  ############\n");
    preRtosPrintRaw = 0;


    vTaskStartScheduler();

    return 0;
}

#define USING_DATA_FLASH
#define USING_PROGRAM_FLASH
/*-----------------------------------------------------------*/
FlashStatus_t data_flash_status = FLASH_ERROR;
FlashStatus_t program_flash_status = FLASH_ERROR;
static void prvSetupHardware(void) {

    vInitializeUARTs(MSS_UART_115200_BAUD);
    printToTerminal("\n\n\n\n\n\n###########  System init...  ############\n");

    delay_cycles(10000);

    printToTerminal("UART INIT\n");
    delay_cycles(10000);

//    MSS_UART_init(&g_mss_uart0, MSS_UART_115200_BAUD, MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    init_WD();
    init_rtc();
    printToTerminal("RTC INIT\n");
    delay_cycles(10000);

    setupHardwareStatus.spi_init = init_spi();

    printToTerminal("SPI INIT\n");
    delay_cycles(10000);

    setupHardwareStatus.can_init = init_CAN(CAN_BAUD_RATE_250K, NULL);
    printToTerminal("CAN INIT\n");
    delay_cycles(10000);

    int test = 0;
    if(test) {
        vTestFlashFull((void *)flash_devices[DATA_FLASH]);
    }
//    init_mram();
    adcs_init_driver();
    printToTerminal("ADCS DRIVER INIT\n");
    delay_cycles(10000);

    setupHardwareStatus.CSP_init = configure_csp();
    printToTerminal("CSP CONFIGURED\n");
    delay_cycles(10000);

    set_csp_init(1);
    printToTerminal("CSP INIT SET\n");
    delay_cycles(10000);


    init_mram();
    printToTerminal("MRAM INIT\n");
    delay_cycles(10000);

    data_flash_status = flash_device_init(flash_devices[DATA_FLASH]);
    printToTerminal("DATA FLASH INIT\n");
    delay_cycles(10000);

    program_flash_status = flash_device_init(flash_devices[PROGRAM_FLASH]);
    printToTerminal("PROGRAM FLASH INIT\n");
    delay_cycles(10000);


    setupHardwareStatus.data_flash_init = data_flash_status;
    setupHardwareStatus.program_flash_init = program_flash_status;

    // TODO make this better - this might crash???
    FilesystemError_t stat = fs_init();
    setupHardwareStatus.fs_init = stat;
    printToTerminal("FILESYSTEM INIT\n");
    delay_cycles(10000);

    printToTerminal("MOUNTING FILESYSTEM\n");
    delay_cycles(10000);

    int err = fs_mount();
    printToTerminal("FILESYSTEM MOUNTED\n");
    delay_cycles(10000);

    
    
    if (err) {
        printToTerminal("\n\n------FILESYSTEM FORMAT REQUIRED------\n");
        printToTerminal("FORMATTING FILESYSTEM\n");
        fs_format();
        printToTerminal("FILESYSTEM FORMATTED\n");
        printToTerminal("MOUNTING FILESYSTEM\n");
        fs_mount();
        printToTerminal("FILESYSTEM MOUNTED\n");
    }

    printToTerminal("OPENING FILES...\n");
    openFiles();
    printToTerminal("FILES OPEN!\n");
}

#define CSP_DEFAULT_PRIORITY 2
#define DEST_ADDRESS 0
#define DEST_PORT 1
#define TIMEOUT_MS 1000
#define OPTIONAL_PARAMS 0

/*-----------------------------------------------------------*/
satPacket packet;
csp_conn_t *txconn;
csp_packet_t *cspPacket;
static void vTestCspClient(void *pvParameters) {
    uint8_t dest;
    vTaskDelay(2000);

    //TODO Add timeout and error checking.
    //TODO Clean up the magic numbers
    printToTerminal("CSP Client started\n");
    while (1) {
        if (xQueueReceive(txQueue, &packet, portMAX_DELAY) == pdTRUE) {
            dest = packet.dest;
            cspPacket = packet.packet;
            txconn = csp_connect(2, 0, dest, 1000, 0);
            csp_send(txconn, cspPacket, 0);
            csp_close(txconn);
            csp_buffer_free(cspPacket);
        }
    }
}

void vTestingTask(void *pvParams) {

    csp_conn_t *conn = NULL;
    csp_packet_t *outPacket = NULL;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void) {
    /* vApplicationMallocFailedHook() will only be called if
     configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
     function that will get called if a call to pvPortMalloc() fails.
     pvPortMalloc() is called internally by the kernel whenever a task, queue,
     timer or semaphore is created.  It is also called by various parts of the
     demo application.  If heap_1.c or heap_2.c are used, then the size of the
     heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
     FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
     to query the size of free heap space that remains (although it does not
     provide information on how the remaining heap might be fragmented). */

    // TODO - Log event!
    printToTerminal("Malloc Failed Hook!\n");
//    taskDISABLE_INTERRUPTS();
//    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {

    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
     task.  It is essential that code added to this hook function never attempts
     to block in any way (for example, call xQueueReceive() with a block time
     specified, or call vTaskDelay()).  If the application makes use of the
     vTaskDelete() API function (as this demo application does) then it is also
     important that vApplicationIdleHook() is permitted to return to its calling
     function, because it is the responsibility of the idle task to clean up
     memory allocated by the kernel to any task that has since been deleted. */

}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void) pcTaskName;
    (void) pxTask;

    /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */

    // TODO - Log event!
    //Try and log to Filesystem if it still works...
    //It doesn't work since this is in an interrupt, so things like the mutex used by FS are broken.
    //So the  flash_write()/flsh_erase() doesn't work either since it calls vTaskDelay.
    //The drivers already skip the delay if the scheduler is not running but I guess that check fails
    //It should definitely be possible to get this working...
//    uint8_t reason=0;
//    getLastRebootReason(&reason);
//    setLastRebootReason(REBOOT_STACK_OVERFLOW | reason);
    // taskDISABLE_INTERRUPTS();
    custom_MSS_UART_polled_tx_string(&g_mss_uart0,
            (const uint8_t*) "Stack Overflow! ", strlen("Stack Overflow! ") + 1);
    custom_MSS_UART_polled_tx_string(&g_mss_uart0,
            (const uint8_t*) pcTaskName, strlen(pcTaskName) + 1);
    //restart the task that overflowed
    for(;;);

}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void) {
    /* This function will be called by each tick interrupt if
     configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
     added here, but the tick hook is called from an interrupt context, so
     code must not attempt to block, and only the interrupt safe FreeRTOS API
     functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/
