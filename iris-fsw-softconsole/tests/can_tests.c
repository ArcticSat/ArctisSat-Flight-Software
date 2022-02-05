//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to CAN bus communication.
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"

#include "drivers/protocol/can.h"


void vTestCANTx(void *pvParameters)
{
    const TickType_t delay = pdMS_TO_TICKS(100);
    CANMessage_t msg = {
                        0x321,
                        8,
                        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
                     };
    for (;;)
    {
        if (CAN_TRANSMIT_READY())
        {
            CAN_transmit_message(&msg);
        }
        vTaskDelay(delay);
    }
}


void vTestCANRx(void *pvParameters)
{
    int messages_processed = 0;
    CANMessage_t rx_msg;
    for (;;)
    {
        if (xQueueReceive(can_rx_queue, &rx_msg, portMAX_DELAY) == pdTRUE)
        {
            messages_processed++;
            // Ground output to terminal
			telemetryPacket_t telemetry={0};
			Calendar_t ts = {0};
			// Send telemetry value
			telemetry.telem_id = POWER_READ_TEMP_ID;
			telemetry.timestamp = ts;
			telemetry.length = 4;
			telemetry.data = rx_msg.data;
			sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);

        }
        else if(uxQueueMessagesWaitingFromISR(can_rx_queue) > 0)
        {
        	telemetryPacket_t telemetry={0};
			Calendar_t ts = {0};
			// Send telemetry value
			telemetry.telem_id = POWER_READ_TEMP_ID;
			telemetry.timestamp = ts;
			telemetry.length = 4;
			telemetry.data = rx_msg.data;
			sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);
        }
        else
        {
    		vTaskDelay(1000);
        }
    }
}
