//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to the adcs system.
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"
#include "tasks/telemetry.h"

#include "drivers/device/adcs_driver.h"
#include "drivers/protocol/uart.h"

void vTestAdcsDriver(void * pvParameters){
    int status = 0;
    int misses = 0;
    int lockout = 1;
    uint8_t *buf[6];
    uint8_t buf_Rx1[32];
    while(1){
        status = pingAdcs();
        if(status != ADCS_DRIVER_NO_ERROR) {
            misses++;
        } else {
            ADCSPingStatus = PING_FOUND;
            misses = 0;
        }

        if(misses > 4) {
            misses = 4;
            ADCSPingStatus = PING_LOST;
        }

//        adcsArbCommand(0x02, buf_Rx1, 0x0D);
//        logADCSTelem(buf_Rx1, 0x0D);
//        logMessage("\n");
//
//        adcsArbCommand(0x03, buf_Rx1, 0x0B);
//        logADCSTelem(buf_Rx1, 0x0B);
//        logMessage("\n");
//
//
//        adcsArbCommand(0x07, buf_Rx1, 0x03);
//        logADCSTelem(buf_Rx1, 0x03);
//        logMessage("\n");
//
//
//        adcsArbCommand(0x04, buf_Rx1, 0x15);
//        logADCSTelem(buf_Rx1, 0x15);
//        logMessage("\n");
//
//        adcsArbCommand(0x06, buf_Rx1, 0x05);
//        logADCSTelem(buf_Rx1, 0x05);
//        logMessage("\n");

        vTaskDelay(1000);
    }


//    uint8_t telemetryData [ADCS_TELEMETRY_TOTAL_SIZE] = {0xFF};
//    uint8_t telemetryData2 [ADCS_MAGNETORQUER_DATA_SIZE] = {0xFF};
//    while(1){
//
//        AdcsDriverError_t result = adcs_power_on();
//        if(!result){
//            while(1);
//        }
//
//        result = adcs_reset();
//        if(!result){
//            while(1);
//        }
//
//        result = adcs_initiate_telemetry();
//        if(!result){
//            while(1);
//        }
//
//
//        result = adcs_read_telemetry(telemetryData);
//        if(!result){
//            while(1);
//        }
//        //Verify the telemetry data here.
//        uint8_t pwm_cycle = 128; // This gives a 50% duty cycle.
//        result = adcs_turn_on_magnetorquer(MAGNETORQUER_X, pwm_cycle);
//        if(!result){
//            while(1);
//        }
//       result = adcs_turn_on_magnetorquer(MAGNETORQUER_Y,pwm_cycle);
//        if(!result){
//            while(1);
//        }
//        result = adcs_turn_on_magnetorquer(MAGNETORQUER_Z,pwm_cycle);
//        if(!result){
//            while(1);
//        }
//
//        result = adcs_turn_off_magnetorquer(MAGNETORQUER_X);
//        if(!result){
//            while(1);
//        }
//        result = adcs_turn_off_magnetorquer(MAGNETORQUER_Y);
//        if(!result){
//            while(1);
//        }
//        result = adcs_turn_off_magnetorquer(MAGNETORQUER_Z);
//        if(!result){
//            while(1);
//        }
//
//
//        result = adcs_read_magnetorquer_data(telemetryData2);
//        if(!result){
//            while(1);
//        }
//        //Verify the telemetry data here.
//
//        vTaskDelay(pdMS_TO_TICKS(2500)); // Repeat the test every 2.5 seconds.
//    }

}
