//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Driver interface for the file system.
//
// History
// 2020-04-10 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/semphr.h>
#include "drivers/filesystem_driver.h"
#include "drivers/device/memory/flash_common.h"
#include <stdint.h>


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


/*** The following four parameters are verified to work for (at least)
 * the following devices:
 * - AT25SF_DATA_FLASH
 * - W25Q_PROGRAM_FLASH
 */
//#define FS_READ_SIZE      1
//#define FS_PROG_SIZE		256
//#define FS_BLOCK_SIZE		(2 * 2048)
//#define FS_CACHE_SIZE		256
//#define FS_LOOKAHEAD_SIZE   2048  //Should probably be much smaller...

//The following parameters seems to work for the W25N.
#define FS_READ_SIZE        512
#define FS_PROG_SIZE		512
#define FS_BLOCK_SIZE		(64 * 2048) //Must be in brackets since we divide by this in the code!
#define FS_CACHE_SIZE		512

#define FS_LOOKAHEAD_SIZE	512
#define FS_BLOCK_CYCLES		500

#define FS_MAX_OPEN_FILES	3

//For testing only. This will offset the location where filesystem is mounted.
//Set to 0!
#define FS_MOUNT_OFFSET     (0x0)

#define FS_FLASH_DEVICE	DATA_FLASH
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTURES AND STRUCTURE TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//		Read a region in a block. Negative error codes are propagated to the user.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int fs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//		Program a region in a block. The block must have previously
//		been erased. Negative error codes are propagated to the user. May return
//		LFS_ERR_CORRUPT if the block should be  considered bad.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int fs_prog(const struct lfs_config *c, lfs_block_t block,lfs_off_t off, const void *buffer, lfs_size_t size);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
// 		Erase a block. A block must be erased before being programmed.
// 		The state of an erased block is undefined. Negative error codes
// 		are propagated to the user.
// 		May return LFS_ERR_CORRUPT if the block should be considered bad.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int fs_erase(const struct lfs_config *c, lfs_block_t block);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
// 		Sync the state of the underlying block device. Negative error codes
// 		are propagated to the user.
//
//		See: https://github.com/ARMmbed/littlefs/issues/408
//		The sync callback marks the point in time where littlefs needs pending
//		write operations to be reflected on disk.
//		Though it's worth noting that polling during prog/erase may be easier to implement. <-- We're gonna do this!
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
 int fs_sync(const struct lfs_config *c);


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBALS AND FILE_SCOPE VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

int g_fs_status = FS_ERR_EXIST;

//Buffers
static uint8_t fs_lookahead_buffer[FS_LOOKAHEAD_SIZE];
static uint8_t fs_read_buffer[FS_CACHE_SIZE];
static uint8_t fs_write_buffer[FS_CACHE_SIZE];
static uint8_t file_buffers[FS_MAX_OPEN_FILES][FS_CACHE_SIZE];
static struct lfs_file_config file_configs[FS_MAX_OPEN_FILES];
static uint8_t open_files;


static lfs_t lfs;
static struct lfs_config config = {	.read  = fs_read,
											.prog  = fs_prog,
											.erase = fs_erase,
											.sync  = fs_sync,

											// block device configuration
											.read_size = FS_READ_SIZE,
											.prog_size = FS_PROG_SIZE,
											.block_size = FS_BLOCK_SIZE,		//Erase size.

											.cache_size = FS_CACHE_SIZE, 		//Must be a multiple of the read and program sizes, and a factor of the block size
											.lookahead_size = FS_LOOKAHEAD_SIZE, //The lookahead buffer is stored as a compact bitmap, so each byte of RAM can track 8 blocks. Must be a multiple of 8.
											.block_cycles = FS_BLOCK_CYCLES,

											.lookahead_buffer = fs_lookahead_buffer,
											.read_buffer = fs_read_buffer,
											.prog_buffer = fs_write_buffer };

static SemaphoreHandle_t fs_lock_handle;


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

int get_fs_status(void)
{
	return g_fs_status;
}

void filesystem_initialization(void)
{
	// Initialize littlefs
	g_fs_status = fs_init();
    //Mount the file system.
	g_fs_status = fs_mount();
    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (g_fs_status != FS_OK) {
    	g_fs_status = fs_format(); //Is there anything else we should do or try first?
    	g_fs_status = fs_mount();
    }
    // Update boot count, if initialization okay
    if(g_fs_status == FS_OK){
		// update boot count
        int result_fs = 1;
		lfs_file_t file = {0}; //Set to 0 because debugger tries to read fields of struct one of which is a pointer, but since this is on free rtos heap, initial value is a5a5a5a5.
	    uint32_t boot_count = 0;
		result_fs = fs_file_open( &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
		result_fs = fs_file_read( &file, &boot_count, sizeof(boot_count));
		boot_count += 1;
		result_fs = fs_file_rewind( &file);
		result_fs = fs_file_write( &file, &boot_count, sizeof(boot_count));
		// remember the storage is not updated until the file is closed successfully
		result_fs = fs_file_close( &file);
    }
}


int fs_list_dir(char * path,int recursive){
//From here: https://github.com/littlefs-project/littlefs/issues/542

    lfs_dir_t dir;
    struct lfs_info info;
    int err = lfs_dir_open(&lfs, &dir, "/");
    if (err) {
        return err;
    }

    while (true) {
        int res = lfs_dir_read(&lfs, &dir, &info);
        if (res < 0) {
            lfs_dir_close(&lfs, &dir);
            return err;
        }

        if (!res) {
            break;
        }

        printf("%s %d %d", info.name, info.type, info.size);
    }

    err = lfs_dir_close(&lfs, &dir);
    if (err) {
        return err;
    }
    return err;
}

int fs_is_open(lfs_file_t * file){

    return -1; //lfs_mlist_isopen(lfs.mlist, (struct lfs_mlist*)file);
}

int fs_file_exist(char * path){
    lfs_file_t file = {0};
    int exist = fs_file_open(&file, path, LFS_O_RDONLY);
    if(exist <0){
        return 0;
    }else{
        fs_file_close(&file);
        return 1;
    }
}

int fs_file_size_from_path(char * path){

    lfs_file_t file = {0};
    struct lfs_info info ={0};

    fs_stat(path, &info);
    return info.size;
}

int fs_copy_file(char * filePath, char * newPath){

    lfs_file_t file = {0};
    lfs_file_t newfile = {0};

    int result_fs = fs_file_open( &file, filePath, LFS_O_RDONLY);
    if(result_fs < 0){
        printf("%s: Could not open file %s: %d\n",__FUNCTION__,filePath,result_fs);
        return result_fs;
    }


    result_fs = fs_file_open( &newfile, newPath, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if(result_fs < 0){
        printf("%s: Could not open file %s: %d\n",__FUNCTION__,newPath,result_fs);
        return result_fs;
    }

    //I think littefs already uses 256b cache so anything bigger shouldn't affect speedup...
    //But larger will need more task stack.
    uint8_t buf[256]={0};
    int read = 0;
    //Now just transfer the data:
    while((read=fs_file_read(&file, buf, 256))>0){

        fs_file_write(&newfile, buf, read);
    }

    result_fs = fs_file_close(&file);
    if(result_fs < 0){
           printf("%s: Could not close file %s: %d\n",__FUNCTION__,filePath,result_fs);
           return result_fs;
    }

    result_fs = fs_file_close(&newfile);
    if(result_fs < 0){
           printf("%s: Could not close file %s: %d\n",__FUNCTION__,newPath,result_fs);
           return result_fs;
    }

    return 0;
}

uint32_t fs_free_space(){

	lfs_ssize_t blocks_used = fs_size();
    uint32_t free_space = (config.block_count-blocks_used)*(config.block_size);
    return free_space;
}

 FilesystemError_t fs_init(){

	 FilesystemError_t result = FS_OK;

	 open_files = 0;
	 //Get the total number of blocks by dividing the device byte count by the block byte count.
	 config.block_count = (flash_devices[DATA_FLASH]->device_size-FS_MOUNT_OFFSET)/FS_BLOCK_SIZE;

	 //Setup the mutex. See https://github.com/ARMmbed/littlefs/issues/156 and
	 //						https://github.com/ARMmbed/littlefs/pull/317
	 fs_lock_handle = xSemaphoreCreateRecursiveMutex();
	 if(fs_lock_handle == NULL){
		 result =  FS_ERR_LOCK;
	 }

	 return result;
 }

int fs_format(){

    int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_format(&lfs, &config);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_mount(){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_mount(&lfs, &config);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_unmount(){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result =  lfs_unmount(&lfs);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_remove( const char *path){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_remove(&lfs, path);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}
	 return result;
}

int fs_rename( const char *oldpath, const char *newpath){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_rename(&lfs, oldpath, newpath);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}
	 return result;
}

int fs_stat( const char *path, struct lfs_info *info){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_stat(&lfs, path, info);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}
	 return result;
}

lfs_ssize_t fs_getattr( const char *path, uint8_t type, void *buffer, lfs_size_t size){
	lfs_ssize_t result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_getattr(&lfs, path, type, buffer, size);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}
	 return result;
}

int fs_setattr( const char *path, uint8_t type, const void *buffer, lfs_size_t size){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_setattr(&lfs, path, type, buffer, size);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}
	 return result;
}

int fs_removeattr( const char *path, uint8_t type){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		result = lfs_removeattr(&lfs, path, type);
		xSemaphoreGiveRecursive(fs_lock_handle);
	}
	 return result;
}

int fs_file_open( lfs_file_t *file, const char *path, int flags){

	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){

		if(open_files<FS_MAX_OPEN_FILES){

			file_configs[open_files].attr_count = 0;
			file_configs[open_files].attrs = 0;
			file_configs[open_files].buffer = file_buffers[open_files];

			result = lfs_file_opencfg(&lfs, file, path, flags, &file_configs[open_files]);

			if(result >=0){
				//File open was successful, increment number of open files.
				open_files += 1;
			}
		}
		else{
			result = FS_ERR_OPENFILES;
		}

		xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_file_close( lfs_file_t *file){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){

		result = lfs_file_close(&lfs, file);

		if(result >=0){
			//File close was successful, decrement number of open files.
			open_files -= 1;

			//TODO:Not sure if we should clear the file buffer. Check if lfs does this.
		}

		xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_file_sync( lfs_file_t *file){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_file_sync(&lfs, file);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

lfs_ssize_t fs_file_read( lfs_file_t *file, void *buffer, lfs_size_t size){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_file_read(&lfs, file, buffer, size);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

lfs_ssize_t fs_file_write( lfs_file_t *file, const void *buffer, lfs_size_t size){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_file_write(&lfs, file, buffer, size);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

lfs_soff_t fs_file_seek( lfs_file_t *file, lfs_soff_t off, int whence){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_file_seek(&lfs, file, off, whence);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_file_truncate( lfs_file_t *file, lfs_off_t size){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_file_truncate(&lfs, file, size);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

lfs_soff_t fs_file_tell( lfs_file_t *file){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_file_tell(&lfs, file);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_file_rewind( lfs_file_t *file){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_file_rewind(&lfs, file);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

 lfs_soff_t fs_file_size( lfs_file_t *file){
		lfs_soff_t result = FS_ERR_LOCK;

		if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
			 result =  lfs_file_size(&lfs, file);
			 xSemaphoreGiveRecursive(fs_lock_handle);
		}

		return result;
}

int fs_mkdir( const char *path){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_mkdir(&lfs, path);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_dir_open( lfs_dir_t *dir, const char *path){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_dir_open(&lfs, dir, path);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_dir_close( lfs_dir_t *dir){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_dir_close(&lfs, dir);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_dir_read( lfs_dir_t *dir, struct lfs_info *info){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_dir_read(&lfs, dir, info);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
 }

int fs_dir_seek( lfs_dir_t *dir, lfs_off_t off){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_dir_seek(&lfs, dir, off);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

lfs_soff_t fs_dir_tell( lfs_dir_t *dir){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_dir_tell(&lfs, dir);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_dir_rewind( lfs_dir_t *dir){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_dir_rewind(&lfs, dir);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

lfs_ssize_t fs_size(){
	lfs_ssize_t result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_fs_size(&lfs);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}

int fs_traverse( int (*cb)(void*, lfs_block_t), void *data){
	int result = FS_ERR_LOCK;

	if(xSemaphoreTakeRecursive(fs_lock_handle,portMAX_DELAY) == pdTRUE){
		 result =  lfs_fs_traverse(&lfs, cb, data);
		 xSemaphoreGiveRecursive(fs_lock_handle);
	}

	return result;
}



int fs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size){

	int result = FS_OK;

	uint32_t addr = (block*FS_BLOCK_SIZE)+off + FS_MOUNT_OFFSET;

	FlashStatus_t stat = flash_read(flash_devices[FS_FLASH_DEVICE], addr, buffer, size);

	//TODO: Pass through flash error value.
	if(stat != FLASH_OK){
		result = FS_ERR_IO;
	}

	return result;

}

int fs_prog(const struct lfs_config *c, lfs_block_t block,lfs_off_t off, const void *buffer, lfs_size_t size){

	int result = FS_OK;

	uint32_t addr = (block*FS_BLOCK_SIZE)+off+FS_MOUNT_OFFSET;

	FlashStatus_t stat = flash_write(flash_devices[FS_FLASH_DEVICE], addr, buffer, size);

	//TODO: Pass through flash error value.
	if(stat != FLASH_OK){
		result = FS_ERR_IO;
	}

	return result;
}

int fs_erase(const struct lfs_config *c, lfs_block_t block){

	int result = FS_OK;

	uint32_t addr = (block*FS_BLOCK_SIZE)+FS_MOUNT_OFFSET;

	FlashStatus_t stat = flash_erase(flash_devices[FS_FLASH_DEVICE], addr);

	//TODO: Pass through flash error value.
	if(stat != FLASH_OK){
		result = FS_ERR_IO;
	}

	return result;

}

int fs_sync(const struct lfs_config *c){
	//Since our flash read/write/erase functions are blocking,
	//then there should be no need to sync, since any operations will be reflected
	// on the flash devices as soon as the write/erase functions return.
	return FS_OK;
}
