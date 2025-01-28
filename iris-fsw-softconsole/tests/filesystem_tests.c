//-------------------------------------------------------------------------------------------------
// File Description:
//  This file contains tests related to the file system. Additional tests
//  are in the littleFS folder in the Libraries folder.
//
// History
// 2020-04-21 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/task.h>
#include "tests.h"

#include "drivers/filesystem_driver.h"

lfs_file_t CDH_telem = { 0 }; //Set to 0 because debugger tries to read fields of struct one of which is a pointer, but since this is on free rtos heap, initial value is a5a5a5a5.
lfs_file_t AODCS_telem = { 0 };
lfs_file_t POWER_telem = { 0 };
lfs_file_t PAYLOAD_telem = { 0 };
lfs_file_t COMMS_telem = { 0 };
lfs_file_t BOOTCOUNT = { 0 };

lfs_file_t files[6];

char names[6][15];

void vTestFS(void *pvParams) {


    FilesystemError_t stat = fs_init();

    files[0] = CDH_telem;
    files[1] = AODCS_telem;
    files[2] = POWER_telem;
    files[3] = PAYLOAD_telem;
    files[4] = COMMS_telem;
    files[5] = BOOTCOUNT;

    strcpy(names[0], "CDH_telem");
    strcpy(names[1], "AODCS_telem");
    strcpy(names[2], "POWER_telem");
    strcpy(names[3], "PAYLOAD_telem");
    strcpy(names[4], "COMMS_telem");
    strcpy(names[5], "BOOTCOUNT");

//    if (stat != FS_OK) {
//        while (1) {
//        }
//    }
//    //Mount the file system.
//    int err = fs_mount();
//
//    // reformat if we can't mount the filesystem
//    // this should only happen on the first boot
//    if (err) {
//        fs_format();
//        fs_mount();
//    }
//
//    int result = 1;
//
//    char readResult[25];


    while(1) {
        vTaskDelay(1000);
    }

//    while (1) {
//
//        uint32_t boot_count = 0;
//        result = fs_file_open(&BOOTCOUNT, "boot_count",
//                LFS_O_RDWR | LFS_O_CREAT);
//        if (result < 0)
//            while (1) {
//            }
//
//        result = fs_file_read(&BOOTCOUNT, &boot_count, sizeof(boot_count));
//        if (result < 0)
//            while (1) {
//            }
//        printf("CDH has started for the %dth time\n", boot_count);
//        // update boot count
//        boot_count += 1;
//        result = fs_file_rewind(&BOOTCOUNT);
//        if (result < 0)
//            while (1) {
//            }
//
//        result = fs_file_write(&BOOTCOUNT, &boot_count, sizeof(boot_count));
//
//        if (result < 0)
//            while (1) {
//            }
//
//        fs_list_dir("", 1);
//
//        // remember the storage is not updated until the file is closed successfully
//        result = fs_file_close(&BOOTCOUNT);
//        char writeString[10];
//        for(int i = 0; i < 5; i++) {
//            fs_file_open(&files[i], names[i], LFS_O_RDWR | LFS_O_CREAT);
//            fs_file_read(&files[i], &readResult[0], 8);
//            fs_file_rewind(&files[i]);
//            sprintf(&writeString[0], "TEST %d", i);
//            fs_file_write(&files[i], &writeString, strlen(writeString));
//            fs_file_close(&files[i]);
//        }
//    }
}
