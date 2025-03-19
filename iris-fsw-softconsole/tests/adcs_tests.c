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


#include <stdint.h>

uint16_t crcTable[] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t crcValue = 0xFFFF;

uint16_t calculateCRC(uint8_t byte) {
    crcValue = (crcValue << 8) ^ crcTable[((crcValue >> 8) ^ byte) & 0x00FF];
    return crcValue;
}

uint8_t testData[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};

int talkToWheels() {
    uint16_t crcRes;
    for(int i = 0; i < 9; i++) {
            crcRes = calculateCRC(testData[i]);
//            printf("%X\n", crcRes);
    }
    // printf("test");

    uint8_t rxData[128];
    uint8_t txData[128];

    //D0 07 00 00 E8 03
    txData[0] = 0x7E;


    txData[1] = 0x06;

    txData[2] = 0x30;
    txData[3] = 0x75;
    txData[4] = 0x00;
    txData[5] = 0x00;

    txData[6] = 0x88;
    txData[7] = 0x13;

    txData[8] = 0xD5;
    txData[9] = 0xF1;

    txData[10] = 0x7E;

    adcsTxRx(txData, 11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    adcsTxRx(NULL, 0, rxData, 11);

    txData[0] = 0x7E;


    txData[1] = 0x04;

    txData[2] = 0x74;
    txData[3] = 0xA1;

    txData[4] = 0x7E;


    adcsTxRx(txData, 5, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    adcsTxRx(NULL, 0, rxData, 15);

    txData[0] = 0x7E;


    txData[1] = 0x06;

    txData[2] = 0x00;
    txData[3] = 0x00;
    txData[4] = 0x00;
    txData[5] = 0x00;

    txData[6] = 0x64;
    txData[7] = 0x00;

    txData[8] = 0x05;
    txData[9] = 0x97;

    txData[10] = 0x7E;


    adcsTxRx(txData, 11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    adcsTxRx(NULL, 0, rxData, 11);


    // printf(calculateCRC(0x09));
    return 0;
}

void vTestAdcsDriver(void * pvParameters){
    int status = 0;
    int misses = 0;
    int lockout = 1;
    uint8_t *buf[6];
    uint8_t buf_Rx1[32];
    while(1){


        talkToWheels();
        vTaskDelay(1000);
//        status = pingAdcs();
//        if(status != ADCS_DRIVER_NO_ERROR) {
//            misses++;
//        } else {
//            ADCSPingStatus = PING_FOUND;
//            misses = 0;
//        }
//
//        if(misses > 4) {
//            misses = 4;
//            ADCSPingStatus = PING_LOST;
//        }

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
