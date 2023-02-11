//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to the external memory (mram/flash).
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"

#include "drivers/device/memory/mram.h"
#include "drivers/device/memory/AS3016204_mram.h"

#include "drivers/device/memory/flash_common.h"
#include <string.h>

#include "drivers/device/memory/mr2xh40_mram.h"

void vTestMRAM(void *pvParameters)
{
    //Write the pins low(0V)
//    MSS_GPIO_set_output(MSS_GPIO_10, 0);
//    MSS_GPIO_set_output(MSS_GPIO_11, 0);
//    vTaskDelay(1000);//Wait 1 second
//    //Write the pins high (3.3V)
//    MSS_GPIO_set_output(MSS_GPIO_10, 1);
//    MSS_GPIO_set_output(MSS_GPIO_11, 1);

    // Test code that writes to all locations of the MRAM, and then reads it back.
    static uint8_t write_buffer[256];
    static uint8_t read_buffer1[sizeof(write_buffer)];
    uint8_t status_reg;
//    uint8_t mram_test_variable;

    static volatile int error_occurred = 0;

    for (int ix = 0; ix < sizeof(write_buffer); ix++)
    {
        write_buffer[ix] = 0x55;
    }
    for(;;)
    {
        // Loop through all addresses.
        for (int ix = 0; ix < AS_MAX_MRAM_ADDR; ix += sizeof(write_buffer))
//    	mr2xh40_read_status_register(&mram_instances[MRAM_INSTANCE_0], &mram_test_variable);

    	// Loop through all addresses.
//        for (int ix = 0; ix < MAX_MRAM_ADDRESS; ix += sizeof(write_buffer))
        {
           for (int ix = 0; ix < sizeof(write_buffer); ix++)
           {
              read_buffer1[ix] = 0xFF;
           }

           vTaskSuspendAll();
          asMram_write( ix, write_buffer, sizeof(write_buffer));
//           mr2xh40_write(&mram_instances[MRAM_INSTANCE_0], ix, write_buffer, sizeof(write_buffer));
           xTaskResumeAll();

           taskYIELD();

           vTaskSuspendAll();
          asMram_read(ix, read_buffer1, sizeof(read_buffer1));
//           mr2xh40_read(&mram_instances[MRAM_INSTANCE_0], ix, read_buffer1, sizeof(read_buffer1));
           xTaskResumeAll();

           for (int iy = 0; iy < sizeof(write_buffer); iy++)
           {
               if (read_buffer1[iy] != write_buffer[iy])
               {
                   error_occurred = 1; // Breakpoint here!
               }
           }


           vTaskDelay(pdMS_TO_TICKS(10)); // Breakpoint here to make sure you are done!
        }
        vTaskSuspend(NULL);}
}

void vTestFlashBB(void *pvParameters){
    //For the W25N only.For fresh chips only. Any writing or erasing could destroy the bb markers.
    //Finds the factory bad blocks, up to 20 bad blocks may be present from the factory.

    // Init
    FlashDev_t * device = (FlashDev_t *) pvParameters;

    //FlashStatus_t result = flash_device_init(device);

    FlashStatus_t result=FLASH_OK;
    uint8_t data_rx[device->page_size];
    memset(data_rx,0,device->page_size);
    int erase_errors=0;

    //Note we are scanning the *blocks* not pages, since the bb markers will be at byte 0 page 0 of each *block*.
    //This makes it much faster (less than 5 min).
    for(int j=0; j< device->device_size;j+=device->erase_size){

        result = flash_read(device,j, data_rx, device->page_size);
        if(data_rx[0] != 0xFF ) {//Just check first location. Only works for fresh chips. We can try checking the first byte spare area for used chips, since our code should not modify that yet??
                erase_errors++;
        }

        memset(data_rx,0,device->page_size);
    }
    while(1){};
}

void vTestFlashFull(void *pvParameters)
{

	// Init
	FlashDev_t * device = (FlashDev_t *) pvParameters;

	FlashStatus_t result = flash_device_init(device);

	if(result != FLASH_OK){
		while(1);
	}

	uint8_t data_tx[device->page_size];
	uint8_t data_rx[device->page_size];
	int erase_errors=0;
	int write_error=0;

	for(int i=0; i< device->page_size;i++){
		data_tx[i]=0;
	}

	// Erase
	FlashStatus_t res=flash_erase_device(device);

    //Verify that the erase device works properly.
    //All addresses should have 0xFF as the data.
    for(int j=0; j< device->device_size;j+=device->page_size){

        flash_read(device,j, data_rx, device->page_size);

        for(int i=0;i<device->page_size;i++){
            if(data_rx[i] != 0xFF) {
            	erase_errors++;}
        }
        memset(data_rx,0,256);
    }

    //Now verify that writing is working:
    //Write to all the addresses on page of data.
    for(int j=0; j<device->device_size;j+=device->page_size){
		//write
		res =flash_write(device,j, data_tx, device->page_size);
		if(res != FLASH_OK) while(1){}

		//Read
		flash_read(device,j, data_rx, device->page_size);

		//Verify
		for(int i=0;i<device->page_size;i++){

			if(data_rx[i] != data_tx[i]){
				write_error++;
			}
		}

		memset(data_rx,0,256);
	}

    for(int j=0; j< device->device_size;j+=device->erase_size){

        flash_erase(device,j);
    }


    //Verify that the erase device works properly.
    //All addresses should have 0xFF as the data.
    for(int j=0; j< device->device_size;j+=device->page_size){

        flash_read(device,j, data_rx, device->page_size);

        for(int i=0;i<device->page_size;i++){
            if(data_rx[i] != 0xFF){
            	erase_errors++;
            }
        }
        memset(data_rx,0,256);
    }

    	while(1){};
}

void vTestFlash(void *pvParameters)
{

	FlashDev_t * device = (FlashDev_t *) pvParameters;

	FlashStatus_t result = flash_device_init(device);

	if(result != FLASH_OK){
		while(1);
	}


	while(1){

	uint8_t data_tx[device->page_size];
	uint8_t data_rx[device->page_size];
	uint8_t data_rx2[device->page_size];
	memset(data_tx,0,device->page_size);
	memset(data_rx,0,device->page_size);
	memset(data_rx2,0,2*device->page_size);

	            //A list of addresses used in this test.
	            uint32_t addr[6] = {0,//First page.
	                                5*device->page_size, //5th page
									6*device->page_size,
									device->erase_size,
									device->device_size - (device->erase_size + device->page_size),
									device->device_size -2*(device->page_size + device->erase_size)
	                                };

	            //Prepare some data to write to each page.
	            for(int i=0;i<device->page_size;i++){
	                data_tx[i] = i;
	            }

	            //Start by erasing the device.
	            FlashStatus_t res = flash_erase_device(device);
	            if(res != FLASH_OK){
	                while(1){}
	            }

	            //Verify that the erase device works properly.
	            //All addresses should have 0xFF as the data.
	            for(int j=0; j<4;j++){

	                flash_read(device,addr[j], data_rx, device->page_size);

	                for(int i=0;i<device->page_size;i++){
	                    if(data_rx[i] != 0xFF) while(1){}
	                }
	                memset(data_rx,0,256);
	            }

	            //Now verify that writing is working:
	            //Write to all the addresses on page of data.

	            for(int j=0; j<4;j++){
	                //write
	                res =flash_write(device,addr[j], data_tx, device->page_size);
	                if(res != FLASH_OK) while(1){}

	                //Read
	                flash_read(device,addr[j], data_rx, device->page_size);

	                //Verify
	                for(int i=0;i<device->page_size;i++){

						if(data_rx[i] != data_tx[i]) while(1){}
	                }

	                memset(data_rx,0,256);
	            }

	            //Make sure we can read more than one page at a time:
	            flash_read(device,addr[1], data_rx2, device->page_size*2);
	            for(int i=0;i<device->page_size*2;i++){

	                if(data_rx2[i] != data_tx[i%device->page_size])
	                    while(1){}
	                }

	            //Test the one of the erase functions

	            res = flash_erase(device,addr[0]);
	            if(res != FLASH_OK) while(1){}

	            //Now check that the only the first address is 0.
	            for(int j=0;j<4;j++){

	                flash_read(device,addr[j], data_rx, device->page_size);

	                for(int i=0;i<device->page_size;i++){
	                    if(j <3){
	                    	if(data_rx[i] != 0xFF) while(1){}
	                    }
	                    else{
	                        if(data_tx[i] != data_rx[i]) while(1){}
	                    }
	                }
	                memset(data_rx,0,256);
	            }


	            vTaskSuspend(NULL);
	}
}
