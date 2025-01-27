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

#include "drivers/device/adcs_driver.h"
#include "drivers/protocol/uart.h"

void vTestAdcsDriver(void * pvParameters){
    int status = 0;
    int misses = 0;
    int lockout = 1;
    uint8_t *buf[6];
    while(1){
//        getGyroMeasurementsGenericRaw(buf);
        status = pingAdcs();
        if(status != ADCS_DRIVER_NO_ERROR) {
            misses++;
        } else {
            if(lockout == 1) {
                lockout = 0;
                char* msg = "ADCS COMM RESTORED\n";
                custom_MSS_UART_polled_tx_string(&g_mss_uart0, msg, strlen(msg));
            }
            misses = 0;
            lockout = 0;
        }

        if(misses > 3 && lockout == 0) {
            lockout = 1;
            char* msg = "ADCS COMM LOST\n";
            custom_MSS_UART_polled_tx_string(&g_mss_uart0, msg, strlen(msg));
        }
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
