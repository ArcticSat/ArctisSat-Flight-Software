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
#include <stdio.h>
#include <string.h>

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
#include "taskhandles.h"
#include "drivers/protocol/can.h"
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



//#define SERVER
//#define CLIENT
//#define CAN_SERVER
#define CSP_SERVER


/* External variables */


//TaskHandles
//This is how we are able to disable/enable tasks, either for operations managment or proper startup order.
TaskHandle_t vTTTScheduler_h;
TaskHandle_t vFw_Update_Mgr_Task_h;
TaskHandle_t vTestCanServer_h;
TaskHandle_t vCSP_Server_h;
TaskHandle_t vTestWD_h;
//Debug Only:
TaskHandle_t vTaskSpinLEDs_h;

extern TaskHandle_t xUART0RxTaskToNotify;


/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );

static void vTestCanServer(void * pvParameters);

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

    //TODO: Are time tagged tasks persistent over restart?
    BaseType_t status;

    /* Prepare the hardware to run this demo. */

     prvSetupHardware();

//    //Make sure FS is up before all tasks
//    FilesystemError_t stat = fs_init();
//        if(stat != FS_OK){
//            //What to do here? try again? can cdh work without fs?
//        }
    // Create LED spinning task
    status = xTaskCreate(    vTaskSpinLEDs,              // The task function that spins the LEDs
                            "LED Spinner",               // Text name for debugging
                            150,                        // Size of the stack allocated for this task
                            NULL,                        // Task parameter is not used
                            1,                           // Task runs at priority 1
                            NULL);                       // Task handle is not used

    // Create UART0 RX Task
    status = xTaskCreate(    vTaskUARTBridge,            // The task function that handles all UART RX events
                            "UART0 Receiver",            // Text name for debugging
                            1200,                        // Size of the stack allocated for this task
                            (void *) &g_mss_uart0,       // Task parameter is the UART instance used by the task
                            3,                           // Task runs at priority 2
                            &xUART0RxTaskToNotify);      // Task handle for task notification
//    status = xTaskCreate(vTTT_Scheduler,
//                         "TTT",
//                         1000,
//                         NULL,
//                         2,
//                         &vTTTScheduler_h);


//    status = xTaskCreate(vTestSPI,
//                         "Test SPI",
//                         1000,
//                         NULL,
//                         1,
//                         NULL);
//
//    status = xTaskCreate(vTestSPI,
//                         "Test SPI2",
//                         1000,
//                         NULL,
//                         1,
//                         NULL);

    // TODO - Starting to run out of heap space for these tasks... should start thinking about
    // increasing heap space or managing memory in a smarter manner. First step would be looking
    // at the FreeRTOS configurations and the linker file *.ld.
//    status = xTaskCreate(vTestCANTx,
//                         "Test CAN Tx",
//                         configMINIMAL_STACK_SIZE,
//                         NULL,
//                         1,
//                         NULL);

//    status = xTaskCreate(vTestCANRx,
//                         "Test CAN Rx",
//                         500,
//                         NULL,
//                         1,
//                         NULL);

#ifdef SERVER
    status = xTaskCreate(vTestCspServer,
                         "Test CSP Server",
                         1000,
                         NULL,
                         1,
                         NULL);

#endif

#ifdef CLIENT
    status = xTaskCreate(vTestCspClient,
                         "Test CSP Client",
                         160,
                         NULL,
                         1,
                         NULL);


#endif
//
//    init_rtc();




#ifdef CSP_SERVER
    status = xTaskCreate(vCSP_Server, "cspServer", 800, NULL, 2, &vCSP_Server_h);
#endif

#ifdef CAN_SERVER
    status = xTaskCreate(vTestCanServer,
                         "Test CAN Rx",
						 1000,
                         NULL,
                         2,
                         &vTestCanServer_h);
//    status = xTaskCreate(vTestCANRx,
//                         "Test CAN Rx",
//                         500,
//                         NULL,
//                         1,
//                         NULL);
#endif

//    status = xTaskCreate(vTestWD,
//                         "Test WD",
//                         configMINIMAL_STACK_SIZE,
//                         NULL,
//                         1,
//                         &vTestWD_h);

//    status = xTaskCreate(vTestFS,
//                         "Test FS",
//                         1000,
//                         NULL,
//                         1,
//                         NULL);

//    status = xTaskCreate(vTestRTC,
//                         "Test RTC",
//                         configMINIMAL_STACK_SIZE,
//                         NULL,
//                         1,
//                         NULL);

    // TR - Not quite sure of the reason, but it appears that when we have a task created for both
    //      vTestRTC and vTestMRAM, the device stops communicating over SPI after the vTestRTC task
    //      finishes transmission (for the first time). In core_spi.c, the software gets stuck in the
    //      while loop "while ( transfer_idx < transfer_size )" on line 134 in "SPI_block_read". The
    //      rx_data_ready variable never evaluates to "true", and so the software is entering an infinite
    //      loop, waiting for the CoreSPI status to be "rx ready" to perform the final read.
//
//    status = xTaskCreate(vTestMRAM,
//                         "Test MRAM",
//                         512,
//                         NULL,
//                         1,
//                         NULL);
//
//	status = xTaskCreate(vTestFlash,
//                         "Test Flash",
//                         2000,
//                         (void *)flash_devices[DATA_FLASH],
//                         1,
//                         NULL);


//    // Task for testing priority queue data structure.
//    status = xTaskCreate(vTaskTest_Priority_Queue,
//    					 "Test Priority_Queue",
//						 256,
//						 NULL,
//						 1,
//						 NULL);
//
//    // Task for testing time tagged task queue.
//status = xTaskCreate(vTestTaskScheduler,
//    					 "Test time tagged task queue",
//						 256,
//						 NULL,
//						 1,
//						 NULL);

//    status = xTaskCreate(vTestADC, "adcTest", 160, NULL, 1, NULL);

//    status = xTaskCreate(vTestAdcsDriver,
//                         "Test ADCS",
//                         configMINIMAL_STACK_SIZE,
//                         NULL,
//                         1,
//                         NULL);

//    status = xTaskCreate(vFw_Update_Mgr_Task,"FwManager",800,NULL,2,&vFw_Update_Mgr_Task_h);
//
//    //Suspend these because csp server will start once csp is up.
//    vTaskSuspend(vFw_Update_Mgr_Task_h);
//    vTaskSuspend(vTTTScheduler_h);
//    vTaskSuspend(vTestCanServer_h);



    vTaskStartScheduler();

    return 0;
}


/*-----------------------------------------------------------*/
static void prvSetupHardware( void )
{
    /* Perform any configuration necessary to use the hardware peripherals on the board. */
    vInitializeLEDs();
//
//    /* UARTs are set for 8 data - no parity - 1 stop bit, see the vInitializeUARTs function to modify
//     * UART 0 set to 115200 to connect to terminal */
    vInitializeUARTs(MSS_UART_115200_BAUD);
//
//    init_WD();
    init_spi();
//    init_rtc();
//    init_mram();
    init_CAN(CAN_BAUD_RATE_250K,NULL);
//    adcs_init_driver();
    flash_device_init(flash_devices[DATA_FLASH]);
    flash_device_init(flash_devices[PROGRAM_FLASH]);
//    initADC();
//    asMram_init();

}



/*-----------------------------------------------------------*/
extern QueueHandle_t can_rx_queue;
uint8_t numCanMsgs = 0;
CANMessage_t can_q[10] = {0};
uint8_t telem_id = 0;
static void vTestCanServer(void * pvParameters)
{
	int messages_processed = 0;
	CANMessage_t rx_msg;
	while(1)
	{
//		BaseType_t q_rec = xQueueReceive(can_rx_queue, &rx_msg, portMAX_DELAY);
//		UBaseType_t numMsgs = uxQueueMessagesWaitingFromISR(can_rx_queue);
//		if (q_rec == pdTRUE)
//		{
//			messages_processed++;
//			// Ground output to terminal
//			telemetryPacket_t telemetry={0};
//			Calendar_t ts = {0};
//			// Send telemetry value
//			telemetry.telem_id = POWER_READ_TEMP_ID;
//			telemetry.timestamp = ts;
//			telemetry.length = 4;
//			telemetry.data = rx_msg.data;
//			sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);
//
//		}

		if(numCanMsgs > 0)
		{
			numCanMsgs--;
			if(numCanMsgs < 0) continue;
			// Check if msg is telem_id
			uint8_t telem_mask = 0;
			int i;
			for(i=0; i < 3; i++) telem_mask |= can_q[numCanMsgs].data[i];
			if(telem_mask == 0)
			{
				telem_id = can_q[numCanMsgs].data[3];
			}
			else
			{
				switch(telem_id)
				{
					case POWER_READ_TEMP_ID:{
						telemetryPacket_t telemetry;
						// Send telemetry value
						telemetry.telem_id = POWER_READ_TEMP_ID;
						telemetry.length = 4;
						telemetry.data = can_q[numCanMsgs].data;
						sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);
						break;
					}
					case POWER_READ_SOLAR_CURRENT_ID:{
						telemetryPacket_t telemetry;
						// Send telemetry value
						telemetry.telem_id = POWER_READ_SOLAR_CURRENT_ID;
						telemetry.length = 4;
						telemetry.data = can_q[numCanMsgs].data;
						sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);
						break;
					}
					case POWER_READ_LOAD_CURRENT_ID:{
						telemetryPacket_t telemetry;
						// Send telemetry value
						telemetry.telem_id = POWER_READ_LOAD_CURRENT_ID;
						telemetry.length = 4;
						telemetry.data = can_q[numCanMsgs].data;
						sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);
						break;
					}
					case POWER_READ_MSB_VOLTAGE_ID:{
						telemetryPacket_t telemetry;
						// Send telemetry value
						telemetry.telem_id = POWER_READ_MSB_VOLTAGE_ID;
						telemetry.length = 4;
						telemetry.data = can_q[numCanMsgs].data;
						sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);
						break;
					}
				}
			}

		}

		vTaskDelay(1000);

	}
}
static void vTestCspServer(void * pvParameters){

	struct csp_can_config can_conf = {0};
    csp_conn_t * conn = NULL;
    csp_packet_t * packet= NULL;
    csp_socket_t * socket  = NULL;


	can_conf.bitrate=250000;
	can_conf.clock_speed=250000;
	can_conf.ifc = "CAN";

	/* Init buffer system with 5 packets of maximum 256 bytes each */
	csp_buffer_init(5, 256);//The 256 number is from the MTU of the CAN interface.

	/* Init CSP with address 0 */
	csp_init(CDH_CSP_ADDRESS);


	/* Init the CAN interface with hardware filtering */
	csp_can_init(CSP_CAN_MASKED, &can_conf);

	/* Setup default route to CAN interface */
	csp_rtable_set(CSP_DEFAULT_ROUTE,0, &csp_if_can,CSP_NODE_MAC);

	size_t freSpace = xPortGetFreeHeapSize();
	/* Start router task with 100 word stack, OS task priority 1 */
	csp_route_start_task(200, 1);


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
/*-----------------------------------------------------------*/
static void vTestCspClient(void * pvParameters){

	struct csp_can_config can_conf;
	can_conf.bitrate=250000;
	can_conf.clock_speed=250000;
	can_conf.ifc = "CAN";

	/* Init buffer system with 5 packets of maximum 256 bytes each */
	csp_buffer_init(5, 256);//The 256 number is from the MTU of the CAN interface.

	/* Init CSP with address 1 */
	csp_init(1);

	/* Init the CAN interface with hardware filtering */
	csp_can_init(CSP_CAN_MASKED, &can_conf);

	/* Setup address 0 to route to CAN interface */
	csp_rtable_set(4,0, &csp_if_can,0);

	size_t freSpace = xPortGetFreeHeapSize();
	/* Start router task with 100 word stack, OS task priority 1 */
	csp_route_start_task(200, 1);


	while(1){
		csp_conn_t * conn;
		csp_packet_t * packet;
		conn = csp_connect(2,4,4,1000,0);	//Create a connection. This tells CSP where to send the data (address and destination port).
		packet = csp_buffer_get(sizeof("Hello World")); // Get a buffer large enough to fit our data. Max size is 256.
		sprintf(packet->data,"Hello World");
		packet->length=strlen("Hello World");
		csp_send(conn,packet,0);
		csp_close(conn);
		vTaskDelay(10000);

//		CANMessage_t msg = {0};
//		msg.id = 0x144;
//		msg.dlc = 8;
//		sprintf(msg.data,"SHEEEESH");
//		CAN_transmit_message(&msg);
//		vTaskDelay(5000);
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
