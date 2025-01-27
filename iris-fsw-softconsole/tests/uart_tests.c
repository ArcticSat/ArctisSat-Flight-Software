//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to UART communication.
//
// History
// 2024-07-08 by Mitesh Patel Ft. Aref Asgari
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include <drivers/filesystem_driver.h>
#include "tests.h"

#include <string.h>

#include "drivers/protocol/uart.h"

#define MAX_IMAGE_BUF 4800

uint8_t cameraImageBuf[MAX_IMAGE_BUF];

#define READ_JPEG 	1
#define READ_RAW 	0
#define SNAP_JPEG	0
#define SNAP_RAW 	1
#define GET_SNAP_FRAME 	1
#define GET_RAW_FRAME 	2
#define GET_JPEG_FRAME 	5
#define SKIP_FRAMES		10
#define PACKAGE_SIZE	16

#define FORMAT_8BIT_GRAY	3
#define FORMAT_16BIT_CrCbY	8
#define FORMAT_16BIT_RGB	6
#define FORMAT_JPEG			7
#define RAW_SIZE_80_60		1
#define RAW_SIZE_160_120 	3
#define RAW_SIZE_128_128 	8
#define RAW_SIZE_128_96 	11
#define JPEG_SIZE_160_128	3
#define JPEG_SIZE_320_240 	5
#define JPEG_SIZE_640_480 	7
#define JPEG_SIZE_128_96 	11

#define HARD_RESET 0
#define STATE_MACHINE_RESET 1



//init Params
// 8-bit Gray Scale (RAW, 8-bit for Y only)	03h
// 16-bit Colour (RAW, CrYCbY)	08h
// 16-bit Colour (RAW, 565(RGB))	06h
// JPEG	07h

// param3
//	80 x 60	01h
//	160 x 120	03h
//	128 x 128	09h
//	128 x 96	0Bh

// param4
//	160 x 128	03h
//	320 x 240	05h
//	640 x 480	07h
//	128 x 96	0Bh

lfs_file_t imageFile = {0};

void vTestUARTTx()
{
    const TickType_t delay = pdMS_TO_TICKS(2000);

//    const uint8_t buf_Tx0[] = {0xc0, 0x0e, 0x11, 0xa1, 0x00, 0x20, 0x00, 0x00, 0x8e, 0x51, 0xc0};
//    const uint8_t buf_Tx1[] = {0xc0, 0x0e, 0x11, 0xa1, 0x00, 0x20, 0x00, 0x00, 0x8e, 0x51, 0xc0};

    const uint8_t buf_Tx0[] = {0xAA, 0x01, 0x00, 0x03, 0x03, 0x03};

    const uint8_t buf_Tx1[] = {0x11, 0x12, 0x13, 0x00};

    uint8_t Tx_Success = 0;
    uint32_t i, useless_number;

    uint8_t buf_Rx0[32];
    uint8_t buf_Rx1[32];


//    const char message[] = "Test";
//    memcpy(buf_Tx, message, sizeof(message));

    FilesystemError_t stat = fs_init();

        if (stat != FS_OK) {
            while (1);
        }
        //Mount the file system.
        int err = fs_mount();

        // reformat if we can't mount the filesystem
		// this should only happen on the first boot
		if (err) {
			fs_format();
			fs_mount();
		}

    for (;;)
    {
    	Tx_Success = 0;

//    	prvUARTSend(&g_mss_uart0, buf_Tx, sizeof(buf_Tx));
//    	Tx_Success = MSS_UART_tx_complete(&g_mss_uart0);
//        vTaskDelay(delay);

//    	MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx);
//    	custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
//    	for( i = 0; i < 2500000; i++)
//    	{
//    		useless_number = 182121*40;
//    	}
//    	MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));
//
//    	for( i = 0; i < 2500000; i++)
//		{
//			useless_number = 182121*40;
//		}

//    	custom_MSS_UART_polled_tx_string(&g_mss_uart1, buf_Tx1, sizeof(buf_Tx1));
//		for( i = 0; i < 2500000; i++)
//		{
//			useless_number = 182121*406;
//		}
//		MSS_UART_get_rx(&g_mss_uart1, buf_Rx1, sizeof(buf_Rx1));

    	/*
    	Tx_Success = syncCamera();
    	if (Tx_Success > 0)
    	{
    		Tx_Success = initCamera();
    		if (Tx_Success > 0)
			{
    			useless_number = 4;
				Tx_Success = takeSnapShot(1, 10);
    			if (Tx_Success > 0)
				{

	    			useless_number = 1;
					Tx_Success = setPackageSize(64);
	    			if (Tx_Success > 0)
					{
						useless_number = 6;
						Tx_Success = getPicture(1);
						if (Tx_Success > 0)
						{
							useless_number = 8;
//							Tx_Success = readImageData(64, cameraImageBuf);

							if (Tx_Success > 0)
							{
								useless_number = 10;

							}else
							{
								useless_number = 11;

							}

						}else
						{
							useless_number = 9;
						}
					}else
					{
						useless_number = 7;
					}

				}else
				{
					useless_number = 5;
				}
			}else
			{
				useless_number = 2;
			}
    	}else
    	{
    		useless_number = 3;
    	}
    	 */

    	Tx_Success = syncCamera();
    	if (Tx_Success > 0)
    	{
    		useless_delay(1000000);
    		Tx_Success = initCamera(FORMAT_JPEG, RAW_SIZE_80_60, JPEG_SIZE_640_480);
    		if (Tx_Success > 0)
			{
    			useless_number = 4;
    			// there should be a delay between init and snapshot
        		useless_delay(10000000);
				Tx_Success = takeSnapShot(SNAP_JPEG, SKIP_FRAMES);
    			if (Tx_Success > 0)
				{

	    			useless_number = 1;
					Tx_Success = setPackageSize(PACKAGE_SIZE);
	    			if (Tx_Success > 0)
					{
						useless_number = 6;
						Tx_Success = getPicture(GET_SNAP_FRAME, READ_JPEG, PACKAGE_SIZE);
						if (Tx_Success > 0)
						{
							useless_number = 8;
//							Tx_Success = readImageData(64, cameraImageBuf);

							if (Tx_Success > 0)
							{
								useless_number = 10;

							}else
							{
								useless_number = 11;

							}

						}else
						{
							useless_number = 9;
						}
					}else
					{
						useless_number = 7;
					}

				}else
				{
					useless_number = 5;
				}
			}else
			{
				useless_number = 2;
			}
    	}else
    	{
    		// if sync fails, RESET camera
    		resetCamera(HARD_RESET);
    		useless_number = 3;
    	}

    	// things to do to get an image
    	// 1- sync with Camera
    	// 2- initialize
    	// 3- set package size
    	// 4- snapshot
    	// 5- get picture
    	// 6- ack the incoming data until the last package is received



    }
}


void useless_delay(unsigned long int uselessDel)
{
    uint32_t i, useless_number;

	for( i = 0; i < uselessDel; i++)
	{
		useless_number = 182121*40;
	}
}


unsigned char resetCamera(unsigned char resetType)
{
	uint8_t buf_Tx0[] = {0xAA, 0x08, 0x00, 0x00, 0x00, 0x00}; // package size command
	uint8_t buf_Rx0[6];

	uint8_t cmdSuccess = 0;

	// params meaning:

	// param2
	// package size (LOW BYTE)

	// param3
	// package size (HIGH BYTE)


	buf_Tx0[2] = resetType; // param1 (reset state machine = 1, hard reset = 0)


	custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
	useless_delay(250000);
	MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));

	if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0E))
	{
		cmdSuccess = 1;
	}

	return cmdSuccess;
}


unsigned char syncCamera()
{
    uint8_t buf_Tx0[] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00}; // sync command
    uint8_t buf_Rx0[6];

	uint8_t syncAck = 0;
	uint8_t syncCount = 0;

	for (syncCount = 0; syncCount < 60; syncCount++)
	{
		custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
		useless_delay(250000);
		MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0)); // this should return ack

		if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0E))
		{
			syncAck = syncCount+1;
			MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0)); // this should return sync

			if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0D))
			{
				buf_Tx0[1] = 0x0E; // send an ack
				buf_Tx0[2] = 0x0D; // send an ack

				custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0)); // this should send ack
				break;
			}
		}
		useless_delay(250000);
	}

	return syncAck;
}


unsigned char initCamera(unsigned char imageFormat, unsigned char resRAW, unsigned char resJPEG)
{
	uint8_t buf_Tx0[] = {0xAA, 0x01, 0x00, 0x00, 0x00, 0x00}; // initialize command
	uint8_t buf_Rx0[6];

	uint8_t initSuccess = 0;

	// params meaning:

	// param2
	// 8-bit Gray Scale (RAW, 8-bit for Y only)	03h
	// 16-bit Colour (RAW, CrYCbY)	08h
	// 16-bit Colour (RAW, 565(RGB))	06h
	// JPEG	07h

	// param3
	//	80 x 60	01h
	//	160 x 120	03h
	//	128 x 128	09h
	//	128 x 96	0Bh

	// param4
	//	160 x 128	03h
	//	320 x 240	05h
	//	640 x 480	07h
	//	128 x 96	0Bh

	buf_Tx0[2] = 0x00; // param1 = 0 (always zero)
	buf_Tx0[3] = imageFormat; // param2 = (image format)
	buf_Tx0[4] = resRAW; // param3 = (RAW resolution)
	buf_Tx0[5] = resJPEG; // param4 = (JPEG resolution)


	custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
	useless_delay(250000);
	MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));

	if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0E))
	{
		initSuccess = 1;
	}

	return initSuccess;
}


unsigned char setPackageSize(unsigned int packSize)
{
	uint8_t buf_Tx0[] = {0xAA, 0x06, 0x00, 0x00, 0x00, 0x00}; // package size command
	uint8_t buf_Rx0[6];

	uint8_t cmdSuccess = 0;

	// params meaning:

	// param2
	// package size (LOW BYTE)

	// param3
	// package size (HIGH BYTE)


	buf_Tx0[2] = 0x08; // param1 = 8 (always 8)
	buf_Tx0[3] = packSize & 0xFF; // param2
	buf_Tx0[4] = (packSize >> 8) & 0xFF; // param3
	buf_Tx0[5] = 0x00; // param4 = 0 (always zero)


	custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
	useless_delay(250000);
	MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));

	if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0E))
	{
		cmdSuccess = 1;
	}

	return cmdSuccess;
}

unsigned char takeSnapShot(unsigned char snapType, unsigned int skipFrames)
{
	uint8_t buf_Tx0[] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00}; // snapshot command
	uint8_t buf_Rx0[6];

	uint8_t cmdSuccess = 0;

	// params meaning:

	// param1
	// JPEG	 0
	// RAW   1

	// param2
	// skip frame (LOW BYTE)

	// param3
	// skip frame (HIGH BYTE)


	buf_Tx0[2] = snapType; // param1 (RAW or JPEG)
	buf_Tx0[3] = skipFrames & 0xFF; // param2
	buf_Tx0[4] = (skipFrames >> 8) & 0xFF; // param3
	buf_Tx0[5] = 0x00; // param4 = 0 (always zero)


	custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
	useless_delay(250000);
	MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));

	if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0E))
	{
		cmdSuccess = 1;
	}

	return cmdSuccess;
}


unsigned char getPicture(unsigned char picType, unsigned char getJPEG, unsigned int packSizeJPEG)
{
	uint8_t buf_Tx0[] = {0xAA, 0x04, 0x00, 0x00, 0x00, 0x00}; // get picture command
	uint8_t buf_Tx0_ACK[] = {0xAA, 0x0E, 0x00, 0x00, 0x00, 0x00}; // get picture command
	uint8_t buf_Rx0[6], buf_tmp, uartRXSuccess, uartFails;
	unsigned long int imageLength=0;
	unsigned int packageID = 0, totalNumPacks = 0;
	uint8_t buf_img_data_package[packSizeJPEG];

	uint8_t cmdSuccess = 0;

	uint32_t dataCnt = 0, thisPackDataLength=0;

	fs_file_open(&imageFile, "imageFile.jpg", LFS_O_RDWR | LFS_O_CREAT);

	// params meaning:

	// param1
	// snapshot	 1
	// RAW   2
	// JPEG	 5

	buf_Tx0[2] = picType; // param1 (Snapshot or RAW or JPEG)
	buf_Tx0[3] = 0x00; // param2 = 0 (always zero)
	buf_Tx0[4] = 0x00; // param3 = 0 (always zero)
	buf_Tx0[5] = 0x00; // param4 = 0 (always zero)

	custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
	useless_delay(4000);
	MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0));

	if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0E))
	{
		if (getJPEG == 0) // read RAW data
			custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0_ACK, sizeof(buf_Tx0_ACK));


		MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0)); // read the first data cmd (6 bytes)
		if (getJPEG == 0) // read RAW data
					{
						dataCnt = 0;
						// start reading image data
						do{
							uartRXSuccess = MSS_UART_get_rx(&g_mss_uart0, &buf_tmp, 1); // read the image data

							if(uartRXSuccess){
			//				memcpy(&cameraImageBuf[dataCnt],  buf_tmp, 48);
							cameraImageBuf[dataCnt] = buf_tmp;
							dataCnt += 1;
			//				useless_delay(100);
							}
						}while(dataCnt < 4800);
						cmdSuccess = 1;

					}

		else if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0A))
		{

			if (getJPEG == 0) // read RAW data
			{
//				dataCnt = 0;
//				// start reading image data
//				do{
//					uartRXSuccess = MSS_UART_get_rx(&g_mss_uart0, &buf_tmp, 1); // read the image data
//
//					if(uartRXSuccess){
//	//				memcpy(&cameraImageBuf[dataCnt],  buf_tmp, 48);
//					cameraImageBuf[dataCnt] = buf_tmp;
//					dataCnt += 1;
//	//				useless_delay(100);
//					}
//				}while(dataCnt < 4800);
//				cmdSuccess = 1;

			}
			else // read the JPEG DATA
			{
				imageLength = (buf_Rx0[3]) | (buf_Rx0[4] << 8) | (buf_Rx0[5] << 16);
				totalNumPacks = imageLength/(packSizeJPEG-6);

				// send first ack
				buf_Tx0_ACK[4] = packageID & 0xFF;
				buf_Tx0_ACK[5] = (packageID >> 8) & 0xFF;
				custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0_ACK, sizeof(buf_Tx0_ACK));
				useless_delay(250000);
				for (;;) // **to be filled with proper for conditions**
				{
					// start reading image data
					uartFails = 0;
					do{
						uartRXSuccess= MSS_UART_get_rx(&g_mss_uart0, buf_img_data_package, sizeof(buf_img_data_package)); // read the image data
						if (uartRXSuccess == 0)
						{
							uartFails++;
							if (uartFails > 250)
							{
								return 0;
							}
						}
					}while(!uartRXSuccess);
					packageID = (buf_img_data_package[0]) | (buf_img_data_package[1] << 8);
					thisPackDataLength = (buf_img_data_package[2]) | (buf_img_data_package[3] << 8);

					buf_Tx0_ACK[4] = packageID & 0xFF;
					buf_Tx0_ACK[5] = (packageID >> 8) & 0xFF;
					custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0_ACK, sizeof(buf_Tx0_ACK));
					useless_delay(250000);

					memcpy(&cameraImageBuf[dataCnt], &buf_img_data_package[4], thisPackDataLength);
//					packageID++;
					dataCnt += thisPackDataLength;

					if (dataCnt >= (MAX_IMAGE_BUF-packSizeJPEG))
					{
						/// almost overflowing the data array so copy everything now before it's overwritten
						fs_file_write(&imageFile, cameraImageBuf, dataCnt);
						dataCnt = 0;
					}

					if(packageID >= totalNumPacks){
						fs_file_close(&imageFile);
						cmdSuccess = 1;
					break;
					}
				}
				cmdSuccess = 1;

		}
	}

	}
	return cmdSuccess;
}

unsigned char readImageData(unsigned int packSize, unsigned char * fullImageData)
{
	uint8_t buf_Tx0[] = {0xAA, 0x0E, 0x00, 0x00, 0x00, 0x00}; // ACK command
	uint8_t buf_Rx0[6];
	uint8_t buf_img_data_package[packSize];
	unsigned long int imageLength=0;
	unsigned int packageID = 0, totalNumPacks = 0;

	uint8_t cmdSuccess = 0;

	// ACK params meaning:

	// param1
	// CMD ID

	// param2
	// ACK counter (don't care)

	// param3
	// package ID (LOW BYTE)

	// param4
	// package ID (HIGH BYTE)


	MSS_UART_get_rx(&g_mss_uart0, buf_Rx0, sizeof(buf_Rx0)); // read the first data cmd (6 bytes)

	if ((buf_Rx0[0] == 0xAA) && (buf_Rx0[1] == 0x0A))
	{
		imageLength = (buf_Rx0[3]) | (buf_Rx0[4] << 8) | (buf_Rx0[5] << 16);
		totalNumPacks = imageLength/(packSize-6);

		// send first ack
		buf_Tx0[4] = packageID & 0xFF;
		buf_Tx0[5] = (packageID >> 8) & 0xFF;
		custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
//		useless_delay(250000);

		// start reading image data

		//for (;;) // **to be filled with proper for conditions**
		{
			// start reading image data
			MSS_UART_get_rx(&g_mss_uart0, fullImageData, 4800); // read the image data
//			packageID = (buf_img_data_package[0]) | (buf_img_data_package[1] << 8);

			buf_Tx0[4] = packageID & 0xFF;
			buf_Tx0[5] = (packageID >> 8) & 0xFF;
//			custom_MSS_UART_polled_tx_string(&g_mss_uart0, buf_Tx0, sizeof(buf_Tx0));
			useless_delay(250000);

//			memcpy(&fullImageData[packageID*packSize], buf_img_data_package, packSize);
			packageID++;

//			if(packageID >= totalNumPacks){
			cmdSuccess = 1;
//			break;
//			}
		}

	}

	return cmdSuccess;
}
