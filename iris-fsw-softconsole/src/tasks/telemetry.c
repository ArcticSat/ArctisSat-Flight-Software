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

uint8_t tempBuff[256]={0};
int csp_init_done =0;
int flashSystemReady = 0;

void unpackTelemetry(uint8_t * data, telemetryPacket_t* output){

    memset(tempBuff,0,256);
    memcpy((void*)&output->timestamp,&data[0],sizeof(Calendar_t));
    memcpy(&output->telem_id, &data[sizeof(Calendar_t)],1);
    memcpy(&output->length, &data[sizeof(Calendar_t)+1],1);
    memcpy(tempBuff,&data[sizeof(Calendar_t)+2],output->length);
    output->data = tempBuff;

}

lfs_soff_t powerWritePos = 0;
lfs_soff_t powerReadPos = 0;


void telemetryManager() {
    char telemBuf[64];
    FilesystemError_t stat = fs_init();

    if (stat != FS_OK) {
        while (1) {
            vTaskDelay(1000);
        }
    }
    //Mount the file system.

    int err = fs_mount();

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        fs_mount();
        fs_format();
    }

    fs_file_open(&powerTelemFile, "powerTelem.log", LFS_O_RDWR | LFS_O_CREAT);
    fs_file_open(&adcsTelemFile, "adcsTelem.log", LFS_O_RDWR | LFS_O_CREAT);
    fs_file_open(&logTelemFile, "logTelem.log", LFS_O_RDWR | LFS_O_CREAT);

    printToTerminal("File system online!");
    flashSystemReady = 1;
    for(;;) {
            if(broadcastTelemFlag) {
                powerWritePos = fs_file_tell(&powerTelemFile);
                fs_file_seek(&powerTelemFile, powerReadPos, 0);
                int result = fs_file_read(&powerTelemFile, telemBuf, 64);
                powerReadPos = 0;
                while(broadcastTelemFlag && result > 0 && powerReadPos < powerWritePos) {
                    sendDataPacket(telemBuf, 64, 0x19);
//                    vTaskDelay(100);
                    result = fs_file_read(&powerTelemFile, telemBuf, 64);
                    powerReadPos = fs_file_tell(&powerTelemFile);
                }
                if(powerReadPos > powerWritePos) powerReadPos = powerWritePos;
                broadcastTelemFlag = 0;
                fs_file_seek(&powerTelemFile, powerWritePos, 0);
            }
        vTaskDelay(500);
    }
}

void logPowerTelem(char* data, int len) {
    if(flashSystemReady) {
        fs_file_write(&powerTelemFile, data, len);
    }
    return;
}

void sendTelemetry(telemetryPacket_t * packet){

            csp_conn_t * conn;
            csp_packet_t * outPacket;
            conn = csp_connect(2,CDH_CSP_ADDRESS,CSP_TELEM_PORT,1000,0);    //Create a connection. This tells CSP where to send the data (address and destination port).
            outPacket = csp_buffer_get(TELEM_HEADER_SIZE+packet->length);

            memcpy(&outPacket->data[0],&packet->timestamp,sizeof(Calendar_t));
            memcpy(&outPacket->data[sizeof(Calendar_t)],&packet->telem_id,1);
            memcpy(&outPacket->data[sizeof(Calendar_t)+1],&packet->length,1);

            //Only do this if there is data.
            if(packet->length){
                memcpy(&outPacket->data[sizeof(Calendar_t)+2],packet->data,packet->length);
            }

            outPacket->length = TELEM_HEADER_SIZE + packet->length;

            int good = csp_send(conn,outPacket,0);
            csp_close(conn);

            if(!good){

                csp_buffer_free(outPacket);
            }
}

void sendTelemetry_direct(telemetryPacket_t * packet,csp_conn_t * conn){

            //csp_conn_t * conn;
            csp_packet_t * outPacket;
            //conn = csp_connect(2,addr,CSP_CMD_PORT,1000,0);   //Create a connection. This tells CSP where to send the data (address and destination port).
            outPacket = csp_buffer_get(TELEM_HEADER_SIZE+packet->length);

            memcpy(&outPacket->data[0],&packet->timestamp,sizeof(Calendar_t));
            memcpy(&outPacket->data[sizeof(Calendar_t)],&packet->telem_id,1);
            memcpy(&outPacket->data[sizeof(Calendar_t)+1],&packet->length,1);

            //Only do this if there is data.
            if(packet->length){
                memcpy(&outPacket->data[sizeof(Calendar_t)+2],packet->data,packet->length);
            }

            outPacket->length = TELEM_HEADER_SIZE + packet->length;

            int good = csp_send(conn,outPacket,0);
            csp_close(conn);

            if(!good){

                csp_buffer_free(outPacket);
            }
}

void sendCommand(telemetryPacket_t * packet,uint8_t addr){


    csp_conn_t * conn;
    csp_packet_t * outPacket;
    conn = csp_connect(2,addr,CSP_CMD_PORT,1000,0);    //Create a connection. This tells CSP where to send the data (address and destination port).
    outPacket = csp_buffer_get(TELEM_HEADER_SIZE+packet->length);

    memcpy(&outPacket->data[0],&packet->timestamp,sizeof(Calendar_t));
    memcpy(&outPacket->data[sizeof(Calendar_t)],&packet->telem_id,1);
    memcpy(&outPacket->data[sizeof(Calendar_t)+1],&packet->length,1);

    //Only do this if there is data.
    if(packet->length){
        memcpy(&outPacket->data[sizeof(Calendar_t)+2],packet->data,packet->length);
    }

    outPacket->length = TELEM_HEADER_SIZE + packet->length;

    int good = csp_send(conn,outPacket,0);
    csp_close(conn);

    if(!good){

        csp_buffer_free(outPacket);
    }
}


void sendTelemetryAddr(telemetryPacket_t * packet,uint8_t addr){

    return;
    csp_conn_t * conn;
    csp_packet_t * outPacket;
    conn = csp_connect(2,addr,CSP_TELEM_PORT,100,0);   //Create a connection. This tells CSP where to send the data (address and destination port).
    outPacket = csp_buffer_get(TELEM_HEADER_SIZE+packet->length);

    memcpy(&outPacket->data[0],&packet->timestamp,sizeof(Calendar_t));
    memcpy(&outPacket->data[sizeof(Calendar_t)],&packet->telem_id,1);
    memcpy(&outPacket->data[sizeof(Calendar_t)+1],&packet->length,1);

    //Only do this if there is data.
    if(packet->length){
        memcpy(&outPacket->data[sizeof(Calendar_t)+2],packet->data,packet->length);
    }

    outPacket->length = TELEM_HEADER_SIZE + packet->length;

    int good = csp_send(conn,outPacket,0);
    csp_close(conn);

    if(!good){

        csp_buffer_free(outPacket);
    }
}
void printMsg(char * msg){
	telemetryPacket_t telemetry;
	Calendar_t time = {0};
	telemetry.timestamp = time;
	telemetry.telem_id = CDH_MSG_ID;
	telemetry.length =  strlen(msg)+1;
	telemetry.data = (uint8_t*) msg;
	sendTelemetryAddr(&telemetry,GROUND_CSP_ADDRESS);
}

int printf(const char *fmt, ...){
    //Based on stuff in this thread:https://electronics.stackexchange.com/questions/206113/how-do-i-use-the-printf-function-on-stm32
    char str[256];


    va_list argp;
    va_start(argp, fmt);

    vsnprintf(str,255,fmt,argp);
    printToTerminal(str);

    return 0;


    if(0 < vsnprintf(str,255,fmt,argp)) // build string
    {
        telemetryPacket_t t;
        Calendar_t now = {0}; //Set to zero, since payload does not keep track of time. CDH will timestamp on receipt.

//      t.telem_id = PAYLOAD_ERROR_ID;
        t.telem_id = CDH_MSG_ID;
        t.timestamp = now;
        t.length = strlen(str) + 1;
        t.data = (uint8_t*)str;

        sendTelemetryAddr(&t, GROUND_CSP_ADDRESS);
    }

    va_end(argp);
    vTaskDelay(100);
    return (strlen(str)>255?255:strlen(str));

}

int is_csp_up(){

    return csp_init_done;
}

void set_csp_init(int state){

    csp_init_done=state;
}

