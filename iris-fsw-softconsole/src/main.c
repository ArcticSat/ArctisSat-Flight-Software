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



//#define SERVER
//#define CLIENT
//#define CAN_SERVER
#define CSP_SERVER


/* External variables */


//TaskHandles
//This is how we are able to disable/enable tasks, either for operations managment or proper startup order.
TaskHandle_t xUART0RxTaskToNotify;
TaskHandle_t vTTTScheduler_h;
TaskHandle_t vFw_Update_Mgr_Task_h;
TaskHandle_t vCanServer_h;
TaskHandle_t vCSP_Server_h;
TaskHandle_t vTestWD_h;
//TaskHandle_t vDetumbleDriver_h;
TaskHandle_t vSunPointing_h;
TaskHandle_t vTestAdcsDriverInterface_h;
//Debug Only:
TaskHandle_t vTaskSpinLEDs_h;


HardwareCheck_t setupHardwareStatus = {0};


/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );

//static void vTestCanServer(void * pvParameters);

static void vTestCspServer(void * pvParameters);
static void vTestCspClient(void * pvParameters);
static void vTestingTask(void * pvParams);

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/*-----------------------------------------------------------*/
/* See the documentation page for this demo on the FreeRTOS.org web site for
full information - including hardware setup requirements. */


int main( void )
{
	// Initialization
   prvSetupHardware();

	// Task Creation
	//TODO: Are time tagged tasks persistent over restart?
	BaseType_t status;

#ifdef MAKER2_DEVKIT_CONFIGURATION
    // Create LED spinning task
    status = xTaskCreate(vTaskSpinLEDs,"LED Spinner",150,NULL,3,NULL);
    status = xTaskCreate(vTaskUARTBridge,"UART0 Receiver",200,(void *) &g_mss_uart0,3,&xUART0RxTaskToNotify);
#endif
//    status = xTaskCreate(vTestWD,"Test WD",configMINIMAL_STACK_SIZE,NULL,3,&vTestWD_h);


//	status = xTaskCreate(vDetumbleDriver,"detumbling",800,NULL,2,&vDetumbleDriver_h);
//	status = xTaskCreate(vSunPointing,"sunpointing",800,NULL,2,&vSunPointing_h);
#ifdef INCLUDE_TASK_TTT
    status = xTaskCreate(vTTT_Scheduler,"TTT",400,NULL,3,&vTTTScheduler_h);
#endif
#ifdef INCLUDE_TASK_CAN_SERVER
#endif
#ifdef INCLUDE_TASK_FW_MANAGER
    status = xTaskCreate(vFw_Update_Mgr_Task,"FwManager",800,NULL,2,&vFw_Update_Mgr_Task_h);
#endif

    // status = xTaskCreate(vTestAdcsDriverInterface,"testAdcs",400,NULL,2,&vTestAdcsDriverInterface_h);

//    //Suspend these because csp server will start once csp is up.
//    vTaskSuspend(vDetumbleDriver_h);
#ifdef INCLUDE_TASK_TTT
    vTaskSuspend(vTTTScheduler_h);
#endif
//    vTaskSuspend(xUART0RxTaskToNotify);
#ifdef INCLUDE_TASK_CAN_SERVER
#endif
#ifdef INCLUDE_TASK_FW_MANAGER
    vTaskSuspend(vFw_Update_Mgr_Task_h);
#endif
//    vTaskSuspend(vTestAdcsDriverInterface_h);
//    vTaskSuspend(vSunPointing_h);

    txQueue = xQueueCreate(5, sizeof(satPacket));

    commsTxQueue = xQueueCreate(10, sizeof(radioPacket_t));
    commsRxQueue = xQueueCreate(10, sizeof(radioPacket_t));


    adcsLogQueue = xQueueCreate(5, sizeof(telemPacket_t));
    powerLogQueue = xQueueCreate(5, sizeof(telemPacket_t));
    logLogQueue = xQueueCreate(5, sizeof(telemPacket_t));

    logMessage("Power on!\n");


    // Start FreeRTOS Tasks
//    status = xTaskCreate(vTestFlashFull,"Test Flash",6000,(void *)flash_devices[DATA_FLASH],1,NULL);
//	status = xTaskCreate(vTestSPI,"Test SPI",1000,NULL,10,NULL);
//	status = xTaskCreate(vTestFlash,"Test Flash",2000,(void *)flash_devices[DATA_FLASH],1,NULL);
    // Create UART0 RX Task


//    // TODO - Starting to run out of heap space for these tasks... should start thinking about
//    // increasing heap space or managing memory in a smarter manner. First step would be looking
//    // at the FreeRTOS configurations and the linker file *.ld.
//    status = xTaskCreate(vTestSPI,"Test SPI",1000,NULL,1,NULL);
//    status = xTaskCreate(vTestSPI,"Test SPI2",1000,NULL,1,NULL);
//    status = xTaskCreate(vTestCANTx,"Test CAN Tx",configMINIMAL_STACK_SIZE,NULL,1,NULL);
//    status = xTaskCreate(vTestCANRx,"Test CAN Rx",500,NULL,10,NULL);
//    status = xTaskCreate(vTestCspServer,"Test CSP Server",1000,NULL,1,NULL);
//    status = xTaskCreate(vTestRTC,"Test RTC",configMINIMAL_STACK_SIZE,NULL,1,NULL);
//    // TR - Not quite sure of the reason, but it appears that when we have a task created for both
//    //      vTestRTC and vTestMRAM, the device stops communicating over SPI after the vTestRTC task
//    //      finishes transmission (for the first time). In core_spi.c, the software gets stuck in the
//    //      while loop "while ( transfer_idx < transfer_size )" on line 134 in "SPI_block_read". The
//    //      rx_data_ready variable never evaluates to "true", and so the software is entering an infinite
//    //      loop, waiting for the CoreSPI status to be "rx ready" to perform the final read.
//    status = xTaskCreate(vTestMRAM,"Test MRAM",512,NULL,1,NULL);
//	status = xTaskCreate(vTestFlashFull,"Test Flash",2000,(void *)flash_devices[PROGRAM_FLASH],1,NULL);
//    status = xTaskCreate(vTestCanServer,"Test CAN Rx",1000,NULL,2,&vTestCanServer_h);
//    // Task for testing priority queue data structure.
//    status = xTaskCreate(vTaskTest_Priority_Queue,"Test Priority_Queue",256,NULL,1,NULL);
//    // Task for testing time tagged task queue.
//    status = xTaskCreate(vTestTaskScheduler,"Test time tagged task queue",256,NULL,1,NULL);
//    status = xTaskCreate(vTestADC, "adcTest", 160, NULL, 1, NULL);
//    status = xTaskCreate(vCanServer,"CAN Rx",1000,NULL,2,&vCanServer_h);



/**THESE ARE THE MAIN FUNCTIONS**/

    radioPacket_t packet;
    int len = strlen("INIT FINISHED!!!");
    packet.len = len + 1;
    packet.header = 0xAA;
    packet.footer = 0xBB;
    memcpy(packet.data, "INIT FINISHED!!!", len);
    packet.type = 0x0A;



//    status = xTaskCreate(vCSP_Server, "cspServer", 500, NULL, 1, &vCSP_Server_h);
//    status = xTaskCreate(vCanServer,"CAN Rx",300,NULL,1,&vCanServer_h);
//    vTaskSuspend(vCanServer_h);
//    status = xTaskCreate(vTestCspClient,"CSP Tx",500,NULL,1,NULL);
//    status = xTaskCreate(commsHandlerTask,"UART Handle",500,NULL,1,NULL);
//    status = xTaskCreate(commsTransmitterTask,"UART Tx",500,NULL,2,NULL);
//    status = xTaskCreate(commsReceiverTask,"UART Rx",500,NULL,2,NULL);
    status = xTaskCreate(vTestAdcsDriver,"ADCS handler",500,NULL,1,NULL);
//    status = xTaskCreate(telemetryManager,"Telem",1000,NULL,1,NULL);

//    status = xTaskCreate(vTestFS,"Test FS",500,NULL,1,NULL);

//    status = xTaskCreate(vTestUARTTx,"Test UART Tx",500,NULL,1,NULL);

//    printToTerminal("Tasks created!\n");
//    printToTerminal("Starting scheduller!");

    vTaskStartScheduler();

    return 0;
}

#define USING_DATA_FLASH
#define USING_PROGRAM_FLASH
/*-----------------------------------------------------------*/
FlashStatus_t data_flash_status 	= FLASH_ERROR;
FlashStatus_t program_flash_status	= FLASH_ERROR;
static void prvSetupHardware( void )
{

    vInitializeUARTs(MSS_UART_115200_BAUD);
//    MSS_UART_init(&g_mss_uart0, MSS_UART_115200_BAUD, MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);



//    init_WD();
//    init_rtc();
    setupHardwareStatus.spi_init = init_spi();
    setupHardwareStatus.can_init = init_CAN(CAN_BAUD_RATE_250K,NULL);
//    init_mram();
//    adcs_init_driver();

#if defined(FLIGHT_MODEL_CONFIGURATION) || defined(ENGINEERING_MODEL_CONFIGURATION)
//    init_mram();
#ifdef USING_DATA_FLASH
    data_flash_status = flash_device_init(flash_devices[DATA_FLASH]);
#endif
#ifdef USING_PROGRAM_FLASH
    program_flash_status =flash_device_init(flash_devices[PROGRAM_FLASH]);
#endif
#endif
    setupHardwareStatus.data_flash_init = data_flash_status;
    setupHardwareStatus.program_flash_init = program_flash_status;

//    FilesystemError_t stat = fs_init();

//        if (stat != FS_OK) {
//            while (1) {
//                int j = 20;
//            }
//        }
//        //Mount the file system.
//
//        int err = fs_mount();
//
////        fs_format();
//
//        // reformat if we can't mount the filesystem
//        // this should only happen on the first boot
//        if (err) {
//            fs_mount();
//            fs_format();
//        }

//    initADC();
//    asMram_init();

//    vTestUARTTx();
//    vTestSPI();

}



/*-----------------------------------------------------------*/
static void vTestCspServer(void * pvParameters){

	struct csp_can_config can_conf = {0};
    csp_conn_t * conn = NULL;
    csp_packet_t * packet= NULL;
    csp_socket_t * socket  = NULL;

//
//	can_conf.bitrate=250000;
//	can_conf.clock_speed=250000;
//	can_conf.ifc = "CAN";
//
//	/* Init buffer system with 5 packets of maximum 256 bytes each */
//	csp_buffer_init(5, 256);//The 256 number is from the MTU of the CAN interface.
//
//	/* Init CSP with address 0 */
//	csp_init(0);
//
//
//	/* Init the CAN interface with hardware filtering */
//	csp_can_init(CSP_CAN_MASKED, &can_conf);
//
//	/* Setup default route to CAN interface */
//	csp_rtable_set(0,0, &csp_if_can,CSP_NODE_MAC);
//
//	size_t freSpace = xPortGetFreeHeapSize();
//	/* Start router task with 100 word stack, OS task priority 1 */
//	csp_route_start_task(200, 1);


	 conn = NULL;
	 packet= NULL;
	socket = csp_socket(0);
	csp_bind(socket, CSP_TELEM_PORT);
	csp_listen(socket,4);

	while(1) {

			conn = csp_accept(socket, 1000);
			if(conn){
				packet = csp_read(conn,0);
				//prvUARTSend(&g_mss_uart0, packet->data, packet->length);
				//printf(“%S\r\n”, packet->data);
				uint32_t addr = packet->id.src;
				if(addr ==  PAYLOAD_CSP_ADDRESS){

				    telemetryPacket_t t = {0};
				    unpackTelemetry(packet->data, &t);

				    double temp = *(double *)t.data;

				    double a = temp;//Does nothing... just a place for breakpoint, while temp is in scope.
				}


				csp_buffer_free(packet);
				csp_close(conn);
			}
	}
}
#define CSP_DEFAULT_PRIORITY 2
#define DEST_ADDRESS 0
#define DEST_PORT 1
#define TIMEOUT_MS 1000
#define OPTIONAL_PARAMS 0

/*-----------------------------------------------------------*/
static void vTestCspClient(void * pvParameters){

    vTaskDelay(2000);
	struct csp_can_config can_conf;
	can_conf.bitrate=250000;
	can_conf.clock_speed=250000;
	can_conf.ifc = "CAN";

//	/* Init buffer system with 5 packets of maximum 256 bytes each */
//	csp_buffer_init(5, 256);//The 256 number is from the MTU of the CAN interface.
//
//	/* Init CSP with address 1 */
//	csp_init(1);
//
//	/* Init the CAN interface with hardware filtering */
//	csp_can_init(CSP_CAN_MASKED, &can_conf);
//
//	/* Setup address 0 to route to CAN interface */
//	csp_rtable_set(4,0, &csp_if_can,0);
//
//	size_t freSpace = xPortGetFreeHeapSize();
//	/* Start router task with 100 word stack, OS task priority 1 */
//	csp_route_start_task(200, 1);

	int allowChange = 0;
    satPacket packet;
    csp_conn_t *txconn;
    csp_packet_t *cspPacket;
    uint8_t dest;
	while(1){
        if(xQueueReceive(txQueue, &packet, portMAX_DELAY) == pdTRUE) {
            dest = packet.dest;
            cspPacket = packet.packet;
            txconn = csp_connect(2, 0, dest, 1000, 0);
            csp_send(txconn, cspPacket, 0);
            csp_close(txconn);
            csp_buffer_free(cspPacket);
        }
	}
}

void vTestingTask(void * pvParams){

    csp_conn_t * conn = NULL;
    csp_packet_t * outPacket = NULL;

    while(1){
        vTaskDelay(pdMS_TO_TICKS(5000));
        static Calendar_t buffer = {
                59u, // seconds
                59u, // minutes
                23u, // hours
                28u, // day
                2u, // February
                20u, // year (2020)
                1u, // weekday
                1u, // week (not used), HOWEVER it must be 1 or greater.
        };


        conn = csp_connect(2,PAYLOAD_CSP_ADDRESS,CSP_CMD_PORT,1000,0);    //Create a connection. This tells CSP where to send the data (address and destination port).
        outPacket = csp_buffer_get(sizeof(Calendar_t)+2);
//        uint8_t testbuff[20];
        TelemetryId_t cmd = PAYLOAD_BOARD_TEMP_ID;
        uint8_t length = 0;///cmd so no data.


//        memcpy(&testbuff[0],&buffer,sizeof(Calendar_t));
//        memcpy(&testbuff[sizeof(Calendar_t)],&cmd,1);
//        memcpy(&testbuff[sizeof(Calendar_t)+1],&length,1);

        memcpy(&outPacket->data[0],&buffer,sizeof(Calendar_t));
        memcpy(&outPacket->data[sizeof(Calendar_t)],&cmd,1);
        memcpy(&outPacket->data[sizeof(Calendar_t)+1],&length,1);
        outPacket->length = sizeof(Calendar_t )+2;

        int good = csp_send(conn,outPacket,0);
        csp_close(conn);

        if(!good){

            csp_buffer_free(outPacket);
        }


    }
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
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

//    taskDISABLE_INTERRUPTS();
//    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{

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


void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

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

    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/
