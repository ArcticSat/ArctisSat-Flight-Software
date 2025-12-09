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

// Overhead for telemetry is size of timestamp + id+length.
#define TELEM_HEADER_SIZE (sizeof(Calendar_t) + 2)

void unpackTelemetry(uint8_t *data, telemetryPacket_t *output)
{
}

Calendar_t currTime;
char timeBuf[32];
uint8_t year, month, day, hour, minute, second;

telemetryPacket_t readTelem;
int readStatus;

lfs_file_t *files[6];

void syncFiles()
{
    // Placeholder for file sync logic
    printToTerminal("Syncing files to storage...\n");
    for (int i = 0; i < 6; i++)
    {
        if (files[i] != NULL)
        {
            fs_file_sync(files[i]);
        }
    }
    printToTerminal("Files synced.\n");
}

void openFiles()
{
    int status;
    status = fs_file_open(&powerFile, "power_telem.log", LFS_O_RDWR | LFS_O_CREAT);
    if (status < 0)
    {
        printToTerminal("Error opening power telem file.\n");
    } else {
        printToTerminal("Power file... OK\n");
    }
    files[0] = &powerFile;
    status = fs_file_open(&adcsFile, "adcs_telem.log", LFS_O_RDWR | LFS_O_CREAT);
    if (status < 0)
    {
        printToTerminal("Error opening adcs telem file.\n");
    } else {
        printToTerminal("ADCS file... OK\n");
    }
    files[1] = &adcsFile;
    status = fs_file_open(&cdhFile, "cdh_telem.log", LFS_O_RDWR | LFS_O_CREAT);
    if (status < 0)
    {
        printToTerminal("Error opening cdh telem file.\n");
    } else {
        printToTerminal("CDH file... OK\n");
    }
    files[2] = &cdhFile;
    status = fs_file_open(&payloadFile, "payload_telem.log", LFS_O_RDWR | LFS_O_CREAT);
    if (status < 0)
    {
        printToTerminal("Error opening payload telem file.\n");
    } else {
        printToTerminal("Payload file... OK\n");
    }
    files[3] = &payloadFile;
    status = fs_file_open(&genericFile, "generic_telem.log", LFS_O_RDWR | LFS_O_CREAT);
    if (status < 0)
    {
        printToTerminal("Error opening generic telem file.\n");
    } else {
        printToTerminal("Generic file... OK\n");
    }
    files[4] = &genericFile;
    status = fs_file_open(&timeTaggedTaskFile, "timeTaggedTasks.dat", LFS_O_RDWR | LFS_O_CREAT);
    if (status < 0)
    {
        printToTerminal("Error opening time tagged tasks file.\n");
    } else {
        printToTerminal("Time-tagged tasks file... OK\n");
    }
    files[5] = &timeTaggedTaskFile;
}

static int fileSystemReady = 0;

void telemetryManager()
{
    mytelemetryPacket_t *pkt;
    lfs_file_t *telemFile;
    lfs_file_t *readFile;
    int fileStatus;

    printToTerminal("File telemetry manager started.\n");
    for (;;)
    {
        requestTelem();
        commitTelemToFile();
        if (telemReadFlag)
        {
            downlinkTelem();
        }
        vTaskDelay(100);
    }
}

void requestTelem() {
    requestTelemPower();
    requestTelemADCS();
    requestTelemPayload();
}

void requestTelemPower() {

}

void requestTelemADCS() {

}

void requestTelemPayload() {

}

void commitTelemToFile()
{
    mytelemetryPacket_t *pkt;
    lfs_file_t *telemFile;
    if (xQueueReceive(telemetryQueue, &pkt, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        size_t totalSize = sizeof(mytelemetryPacket_t) + pkt->length;
        switch (pkt->reporting_device)
        {
        case 0:
            telemFile = &cdhFile;
            break;
        case 1:
            telemFile = &powerFile;
            break;
        case 2:
            telemFile = &adcsFile;
            break;
        case 3:
            telemFile = &cdhFile;
            break;
        case 4:
            telemFile = &payloadFile;
            break;
        default:
            telemFile = &genericFile;
            break;
        }

        if (telemFile->cfg == NULL)
        {
            // file not open?
            printToTerminal("Error: Telemetry file not open, dropping packet.\n");
            vPortFree(pkt);
        }

        fs_file_seek(telemFile, 0, LFS_SEEK_END);

        // write telem to file
        int writeBytes = fs_file_write(telemFile, pkt, totalSize);
        int fileSize = fs_file_size(telemFile);
        printf("Wrote %d bytes to telem file, new size %d bytes.\n", writeBytes, fileSize);
        vPortFree(pkt);
    }
}

void downlinkTelem()
{
    lfs_file_t *readFile;
    if (telemReadFlag)
    {
        char buf[64];
        int fileSize;
        printToTerminal("###BEGIN TELEM DOWNLINK###\n");
        for (int i = 0; i < 5; i++)
        {
            readFile = files[i];
            fs_file_seek(readFile, 0, LFS_SEEK_SET);
            while (1)
            {
                fileSize = fs_file_size(readFile);
                readStatus = fs_file_read(readFile, &buf, 64);
                if (readStatus <= 0)
                {
                    // file is empty, delete and reopen
                    fs_file_truncate(readFile, 0);
                    fs_file_sync(readFile);

                    fileSize = fs_file_size(readFile);
                    break;
                }
                else
                {
                    sendRawData(&buf, readStatus);
                }
            }
        }
    }
    telemReadFlag = 0;
}

void logTelemCustom(char *data, int len, int reporting_device)
{
    // Allocate enough memory for header + data
    mytelemetryPacket_t *pkt = pvPortMalloc(sizeof(mytelemetryPacket_t) + len);
    if (!pkt)
        return;

    ds1393_read_time(&pkt->timestamp);
    pkt->header = 0xABABABAB;
    pkt->reporting_device = reporting_device;
    pkt->telem_id = 2;
    pkt->length = len;

    memcpy(pkt->data, data, len); // copies the string bytes

    volatile char swag = pkt->data[5]; // contains 1s
    swag = data[5];                    // contains 32

    // **Send pointer to queue** (queue holds telemetryPacket_t*)
    xQueueSendToBack(telemetryQueue, &pkt, portMAX_DELAY);
}

void logPowerTelem(char *data, int len)
{
    logTelemCustom(data, len, 1);
}

void logADCSTelem(char *data, int len)
{
    logTelemCustom(data, len, 2);
}

void logMessage(char *data)
{
    mytelemetryPacket_t *pkt = pvPortMalloc(sizeof(mytelemetryPacket_t) + strlen(data) + 1);
    if (!pkt)
    {
        printToTerminal("Failed to allocate memory for telemetry message\n");
        return;
    }

    MSS_RTC_get_calendar_count(&pkt->timestamp);
    pkt->reporting_device = 0;
    pkt->telem_id = 2;
    pkt->length = strlen(data) + 1;

    memcpy(pkt->data, data, pkt->length); // copies the string bytes

    volatile char swag = pkt->data[5]; // contains 1s
    swag = data[5];                    // contains 32

    // **Send pointer to queue** (queue holds telemetryPacket_t*)
    xQueueSendToBack(telemetryQueue, &pkt, portMAX_DELAY);
}

void logTelem(char *data, int len)
{
    return;
}

void sendTelemetry(telemetryPacket_t *packet)
{
}

void sendTelemetry_direct(telemetryPacket_t *packet, csp_conn_t *conn)
{
}

void sendCommand(telemetryPacket_t *packet, uint8_t addr)
{
}

void sendTelemetryAddr(telemetryPacket_t *packet, uint8_t addr)
{
}
void printMsg(char *msg)
{
}

int printf(const char *fmt, ...)
{
    // Based on stuff in this thread:https://electronics.stackexchange.com/questions/206113/how-do-i-use-the-printf-function-on-stm32
    char str[256];

    va_list argp;
    va_start(argp, fmt);

    vsnprintf(str, 255, fmt, argp);
    printToTerminal(str);

    return 0;

    va_end(argp);
    return (strlen(str) > 255 ? 255 : strlen(str));
}

int is_csp_up()
{

}

void set_csp_init(int state)
{

}
