//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  User-facing module for abstracting MRAM initialization and usage. With a chosen MRAM instance, the user can use mr2xh40 driver functions.
//
// History
// 2019-06-09 by Tamkin Rahman
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "drivers/device/memory/mram.h"
#include "drivers/device/memory/AS3016204_mram.h"

#include "board_definitions.h"

#include <string.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define MRAM_CLEAR_CHUNK_SIZE 32

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

// Use this list to maintain the list of MRAMs and the core/slave pins used.
// Note: MRAM_INSTANCE_1 (AS3016204) does not actually use this struct (its driver hardcodes MRAM1_SPI_CORE/MRAM1_SLAVE_CORE internally); the entry
// exists only so both instances can be referred to uniformly by index.
MRAMInstance_t mram_instances[NUM_MRAM_INSTANCES] = {
    //                    core instance,   slave instance
    [MRAM_INSTANCE_0] = { MRAM0_SPI_CORE,  MRAM0_SLAVE_CORE},
    [MRAM_INSTANCE_1] = { MRAM1_SPI_CORE,  MRAM1_SLAVE_CORE}
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void init_mram()
{
    mr2xh40_init(&mram_instances[MRAM_INSTANCE_0]);
    asMram_init();
}

void mram_read(MRAMInstanceIndex_t instance, uint32_t address, uint8_t * rd_buffer, size_t rd_buffer_size)
{
    switch (instance)
    {
        case MRAM_INSTANCE_0:
            mr2xh40_read(&mram_instances[MRAM_INSTANCE_0], address, rd_buffer, rd_buffer_size);
            break;
        case MRAM_INSTANCE_1:
            asMram_read(address, rd_buffer, rd_buffer_size);
            break;
        default:
            break;
    }
}

void mram_write(MRAMInstanceIndex_t instance, uint32_t address, uint8_t * wr_buffer, size_t wr_buffer_size)
{
    switch (instance)
    {
        case MRAM_INSTANCE_0:
            mr2xh40_write(&mram_instances[MRAM_INSTANCE_0], address, wr_buffer, wr_buffer_size);
            break;
        case MRAM_INSTANCE_1:
            asMram_write(address, wr_buffer, wr_buffer_size);
            break;
        default:
            break;
    }
}

void mram_clear(MRAMInstanceIndex_t instance)
{
    uint8_t erase_chunk[MRAM_CLEAR_CHUNK_SIZE];
    memset(erase_chunk, 0xFF, sizeof(erase_chunk));

    uint32_t max_address = (instance == MRAM_INSTANCE_1) ? AS_MAX_MRAM_ADDR : MAX_MRAM_ADDRESS;

    for (uint32_t address = 0; address <= max_address; address += sizeof(erase_chunk))
    {
        uint32_t remaining = (max_address - address) + 1;
        uint32_t chunk_size = (remaining < sizeof(erase_chunk)) ? remaining : sizeof(erase_chunk);
        mram_write(instance, address, erase_chunk, chunk_size);
    }
}
