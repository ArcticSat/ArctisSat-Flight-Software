/*
 * telemetry.c
 *
 *  Created on: Mar. 22, 2021
 *      Author: Joseph Howarth
 */

#include "tasks/telemetry.h"
#include <csp/csp.h>
#include "csp/interfaces/csp_if_can.h"
#include <string.h>

//Overhead for telemetry is size of timestamp + id+length.
#define TELEM_HEADER_SIZE   (sizeof(Calendar_t)+2)

uint8_t tempBuff[256] = { 0 };
int csp_init_done = 0;
volatile uint8_t flashSystemReady;

void unpackTelemetry(uint8_t *data, telemetryPacket_t *output) {

    memset(tempBuff, 0, 256);
    memcpy((void*) &output->timestamp, &data[0], sizeof(Calendar_t));
    memcpy(&output->telem_id, &data[sizeof(Calendar_t)], 1);
    memcpy(&output->length, &data[sizeof(Calendar_t) + 1], 1);
    memcpy(tempBuff, &data[sizeof(Calendar_t) + 2], output->length);
    output->data = tempBuff;

}

Calendar_t currTime;
char timeBuf[32];
uint8_t year, month, day, hour, minute, second;

int telemReadFlag = 0;

telemetryPacket_t readTelem;
lfs_file_t readFile;
int readStatus;
lfs_file_t powerFile;
lfs_file_t adcsFile;
lfs_file_t cdhFile;
lfs_file_t payloadFile;
lfs_file_t genericFile;

void openFiles() {
    fs_file_open(&powerFile, "power_telem.log", LFS_O_RDWR | LFS_O_APPEND | LFS_O_CREAT);
    fs_file_open(&adcsFile, "adcs_telem.log", LFS_O_RDWR | LFS_O_APPEND | LFS_O_CREAT);
    fs_file_open(&cdhFile, "cdh_telem.log", LFS_O_RDWR | LFS_O_APPEND | LFS_O_CREAT);
    fs_file_open(&payloadFile, "payload_telem.log", LFS_O_RDWR | LFS_O_APPEND | LFS_O_CREAT);
    fs_file_open(&genericFile, "generic_telem.log", LFS_O_RDWR | LFS_O_APPEND | LFS_O_CREAT);
}

void telemetryManager() {
    mytelemetryPacket_t *pkt;
    lfs_file_t telemFile;
    int fileStatus;

    openFiles();

    printToTerminal("File system online!");
    for (;;) {
        if(xQueueReceive(telemetryQueue, &pkt, pdMS_TO_TICKS(50)) == pdTRUE) {
            size_t totalSize = sizeof(mytelemetryPacket_t) + pkt->length;
            switch(pkt->reporting_device) {
                case 0:
                    telemFile = cdhFile;
                    break;                
                case 1:
                    telemFile = powerFile;
                    break;
                case 2:
                    telemFile = adcsFile;
                    break;
                case 3:
                    telemFile = cdhFile;
                    break;
                case 4: 
                    telemFile = payloadFile;
                    break;
                default:
                    telemFile = genericFile;
                    break;
            }

            if(telemFile.cfg == NULL) {
                //file not open?
                vPortFree(pkt);
                continue;
            }

            //write telem to file
            fs_file_write(&telemFile, pkt, totalSize);

            vPortFree(pkt);
            // fileStatus = fs_file_close(&telemFile);

        }

        if(telemReadFlag) {
            readFile = powerFile;
            fs_file_rewind(&readFile);
            char buf[64];
            int fileSize;
            while(1) {
                readStatus = fs_file_read(&readFile, &buf, 64);
                fileSize = fs_file_size(&readFile);
                if(readStatus <= 0) {
                    fs_file_truncate(&readFile, 0); //clear file after reading
                    break;
                }
            }
            telemReadFlag = 0;
        }
        vTaskDelay(100);
    }
}

void requestTelem() {
    
}

void logTelemCustom(char* data, int len, int reporting_device) {
    // Allocate enough memory for header + data
        mytelemetryPacket_t *pkt = pvPortMalloc(sizeof(mytelemetryPacket_t) + len);
            if (!pkt) return;

                MSS_RTC_get_calendar_count(&pkt->timestamp);
                    pkt->reporting_device = reporting_device;
                        pkt->telem_id = 2;
                            pkt->length = len;

                                memcpy(pkt->data, data, len);  // copies the string bytes

                                    volatile char swag = pkt->data[5]; //contains 1s
                                        swag = data[5]; //contains 32

                                            // **Send pointer to queue** (queue holds telemetryPacket_t*)
                                                xQueueSendToBack(telemetryQueue, &pkt, portMAX_DELAY);
}

void logPowerTelem(char *data, int len) {
    logTelemCustom(data, len, 1);
}

void logADCSTelem(char *data, int len) {
    logTelemCustom(data, len, 2);
}

void logMessage(char *data) {
    mytelemetryPacket_t *pkt = pvPortMalloc(sizeof(mytelemetryPacket_t) + strlen(data) + 1);
    if (!pkt) return;

    MSS_RTC_get_calendar_count(&pkt->timestamp);
    pkt->reporting_device = 0;
    pkt->telem_id = 2;
    pkt->length = strlen(data) + 1;

    memcpy(pkt->data, data, pkt->length);  // copies the string bytes

    volatile char swag = pkt->data[5]; //contains 1s
    swag = data[5]; //contains 32

    // **Send pointer to queue** (queue holds telemetryPacket_t*)
    xQueueSendToBack(telemetryQueue, &pkt, portMAX_DELAY);
}

void logTelem(char *data, int len) {
    return;
}

void sendTelemetry(telemetryPacket_t *packet) {

}

void sendTelemetry_direct(telemetryPacket_t *packet, csp_conn_t *conn) {

}

void sendCommand(telemetryPacket_t *packet, uint8_t addr) {

}

void sendTelemetryAddr(telemetryPacket_t *packet, uint8_t addr) {

}
void printMsg(char *msg) {
    telemetryPacket_t telemetry;
    Calendar_t time = { 0 };
    telemetry.timestamp = time;
    telemetry.telem_id = CDH_MSG_ID;
    telemetry.length = strlen(msg) + 1;
    telemetry.data = (uint8_t*) msg;
    sendTelemetryAddr(&telemetry, GROUND_CSP_ADDRESS);
}

int printf(const char *fmt, ...) {
    //Based on stuff in this thread:https://electronics.stackexchange.com/questions/206113/how-do-i-use-the-printf-function-on-stm32
    char str[256];

    va_list argp;
    va_start(argp, fmt);

    vsnprintf(str, 255, fmt, argp);
//    printToTerminal(str);

    return 0;

    if (0 < vsnprintf(str, 255, fmt, argp)) // build string
            {
        telemetryPacket_t t;
        Calendar_t now = { 0 }; //Set to zero, since payload does not keep track of time. CDH will timestamp on receipt.

//      t.telem_id = PAYLOAD_ERROR_ID;
        t.telem_id = CDH_MSG_ID;
        t.timestamp = now;
        t.length = strlen(str) + 1;
        t.data = (uint8_t*) str;

        sendTelemetryAddr(&t, GROUND_CSP_ADDRESS);
    }

    va_end(argp);
    vTaskDelay(100);
    return (strlen(str) > 255 ? 255 : strlen(str));

}

int is_csp_up() {

    return csp_init_done;
}

void set_csp_init(int state) {

    csp_init_done = state;
}

