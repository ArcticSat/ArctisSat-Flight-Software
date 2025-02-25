#ifndef ALL_TESTS_H
#define ALL_TESTS_H
//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains all the tests for the MBSat flight software.
//
// History
// 2020-04-21 by Joseph Howarth 
// - Created.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//      Unit tests
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//      Onboard Tests
//-------------------------------------------------------------------------------------------------

//FreeRTOS Test task for CoreSPI
void vTestSPI();

//FreeRTOS Test task for CAN
void vTestCANTx(void *pvParameters);
void vTestCANRx(void *pvParameters);

//FreeRTOS Test task for UART
void vTestUARTTx();


//FreeRTOS Test task for Watchdog.
void vTestWD(void *pvParameters);

//FreeRTOS Test task for RTC.
void vTestRTC(void *pvParameters);

//FreeRTOS Test task for MRAM
void vTestMRAM(void *pvParameters);

//FreeRTOS Test task for external flash memory.
void vTestFlash(void *pvParameters);
void vTestFlashFull(void *pvParameters);
//For the W25N only.Finds the factory bad blocks.For fresh chips only. Any writing or erasing could destroy the bb markers.
void vTestFlashBB(void *pvParameters);

//FreeRTOS Test task for ADCS driver.
void vTestAdcsDriver(void * pvParameters);

//FreeRTOS Test task for file system.
void vTestFS(void * pvParams);

//Tests the onboard ADC measuring the board temp.
void vTestADC(void * pvParams);

unsigned char resetCamera(unsigned char resetType);
unsigned char syncCamera();
void useless_delay(unsigned long int uselessDel);
unsigned char initCamera(unsigned char imageFormat, unsigned char resRAW, unsigned char resJPEG);
unsigned char setPackageSize(unsigned int packSize);
unsigned char takeSnapShot(unsigned char snapType, unsigned int skipFrames);
unsigned char getPicture(unsigned char picType, unsigned char getJPEG, unsigned int packSizeJPEG);
unsigned char readImageData(unsigned int packSize, unsigned char * fullImageData);



#endif
