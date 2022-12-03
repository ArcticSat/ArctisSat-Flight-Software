/*
 * telemetry_manager.c
 *
 *  Created on: Dec. 2, 2022
 *      Author: jpmckoy
 */


#include "application/telemetry_manager.h"
#include "main.h"
#include "drivers/device/memory/flash_common.h"
#include "drivers/device/memory/MT25Q_flash.h"

#define PAGE_SIZE MT25Q_PAGE_SIZE
#define BLOCK_SIZE MT25Q_SUBSECTOR_SMALL_SIZE

typedef enum {
	SC_STATUS_TLM_BLK,
	CDH_TLM_BLK,
	POWER_TLM_BLK,
	PAYLOAD_TLM_BLK,
	ADCS_TLM_BLK,
	NUM_TLM_BLKS,
} TelemetryBlocks;

uint32_t tlm_blk_sizes[NUM_TLM_BLKS] = {
	1, // Spacecraft status
	1, // CDH
	1, // Power
	1, // Payload
	1, // ADCS
};

const uint32_t SC_STATUS_TYPEDEF_SIZE_BYTES = 4 * sizeof(uint32_t);
struct ScStatus_TypeDef {
	// Telemetry status
	uint32_t tlm_blk_base[NUM_TLM_BLKS];
	uint32_t tlm_blk_size[NUM_TLM_BLKS];
	uint32_t tlm_blk_size_max[NUM_TLM_BLKS];
	uint32_t num_packets[NUM_TLM_BLKS];
} sc_status;


void init_telemetry_manager(void)
{
	int i;
	// Initialize spacecraft status struct
	for(i=0; i < NUM_TLM_BLKS; i++){
		sc_status.tlm_blk_base[i] = 0;
		sc_status.tlm_blk_size[i] = 0;
		sc_status.num_packets[i] = 0;
	}
	// Block sizes
	for(i=0; i < NUM_TLM_BLKS; i++){
		sc_status.tlm_blk_size_max[i] = tlm_blk_sizes[i] * BLOCK_SIZE;
	}
	// Block bases
	for(i=1; i < NUM_TLM_BLKS; i++){
		sc_status.tlm_blk_base[i] = sc_status.tlm_blk_base[i-1] + tlm_blk_sizes[i];
	}
}
void log_telemetry(telemetryPacket_t * pkt)
{
	// TBC: Set packet time?!?!
	Calendar_t rx_time = {0};
	pkt->timestamp = rx_time;
	// Get telemetry block from Telemtry ID
	uint8_t tlm_blk_id = 1;
	if(pkt->telem_id < CDH_TELEMETRY_END)
		tlm_blk_id = CDH_TLM_BLK;
	else if(pkt->telem_id < POWER_TELEMETRY_END)
		tlm_blk_id = POWER_TLM_BLK;
	else if(pkt->telem_id < PAYLOAD_TELEMETRY_END)
		tlm_blk_id = PAYLOAD_TLM_BLK;
	else if(pkt->telem_id < ADCS_TELEMETRY_END)
		tlm_blk_id = ADCS_TLM_BLK;
	else
		return;
	// Get write address
	uint32_t address = sc_status.tlm_blk_base[tlm_blk_id] + sc_status.tlm_blk_size[tlm_blk_id];
	// Get write size
//	uint32_t wr_size = sizeof(telemetryPacket_t) + pkt->dlc;
	uint32_t wr_size = 8 + 3 + (uint32_t) pkt->length; // sizeof(telemetryPacket_t) = 11 bytes
	// Update block offset
	sc_status.tlm_blk_size[tlm_blk_id] += wr_size;
	// Wrap block, if full
	if(sc_status.tlm_blk_size[tlm_blk_id] > sc_status.tlm_blk_size_max[tlm_blk_id]){
		// Reset address to top of block
		address = sc_status.tlm_blk_base[tlm_blk_id];
		// Reset block size
		sc_status.tlm_blk_size[tlm_blk_id] = wr_size;
		// Reset number of blocks
		sc_status.num_packets[tlm_blk_id] = 0;
	}
	// Write packet
	flash_write(DATA_FLASH, address, (uint8_t *) pkt, wr_size);
	// Update num packets
	sc_status.num_packets[tlm_blk_id]++;
	// Update spacecraft status
	address = sc_status.tlm_blk_base[SC_STATUS_TLM_BLK];
//	wr_size = sizeof(ScStatus_TypeDef);
	wr_size = SC_STATUS_TYPEDEF_SIZE_BYTES;
	flash_write(DATA_FLASH, address, (uint8_t *) &sc_status, wr_size);
}

void get_telemetry(uint8_t tlm_blk_id)
{
	// Get telemetry block parameters
	uint32_t base_address = sc_status.tlm_blk_base[tlm_blk_id];
	uint32_t rd_size_total = sc_status.tlm_blk_base[tlm_blk_id];
	// Set packet parameters (other than data)
	telemetryPacket_t pkt = {0};
	pkt.telem_id = GND_FRAME_ID;
	Calendar_t tx_time = {0}; // TBC: send time
	pkt.timestamp = tx_time;
	// Get total number of packets needed
	// Sequence number = packet number of frame (4 bytes)
	// Sequence size = total number of packets (4 bytes)
	// # packets needed = rd_size_total / (DLC_MAX - sizeof(seq. num.) - sizeof(seq. size)) + 1
	uint8_t sec_header_size = 2 * sizeof(uint32_t);
	uint8_t read_size_max = DLC_MAX - sec_header_size;
	uint8_t sequence_size = rd_size_total / read_size_max + 1;
	pkt.data[0] = sequence_size;
	// Send frames
	uint8_t rd_size = 0;
	uint32_t rd_offset = 0;
	uint8_t seq_number;
	uint32_t rd_address;
	for(seq_number = 0; seq_number < sequence_size; seq_number++){
		// Sequence number
		pkt.data[1] = seq_number;
		// Read size
		if(rd_size_total - rd_offset > read_size_max){
			rd_size = read_size_max;
		}
		else {
			rd_size = rd_size_total - rd_offset;
		}
		// Read data
		rd_address = base_address + rd_offset;
		flash_read(DATA_FLASH, rd_address, &pkt.data[sec_header_size], rd_size);
		// Send packet
		sendTelemetryAddr(&pkt, GROUND_CSP_ADDRESS);
		// Update offset
		rd_offset += (uint32_t) rd_size;
	}
}







