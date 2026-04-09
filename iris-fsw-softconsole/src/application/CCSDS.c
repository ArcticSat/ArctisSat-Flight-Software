#include <stdint.h>
#include <string.h>
#include "application/CCSDS.h"

#define CCSDS_PRIMARY_HEADER_LEN 6
#define CCSDS_VERSION            0x0  // Always 0 for CCSDS
#define CCSDS_TYPE_TM            0x0  // 0 = TM, 1 = TC
#define CCSDS_TYPE_TC            0x1

/* Build CCSDS primary header */
void ccsds_build_primary_header(
    ccsds_primary_header_t *hdr,
    uint8_t  type,           // TM or TC
    uint8_t  sec_hdr_flag,   // 1 = secondary header present
    uint16_t apid,
    uint8_t  seq_flags,      // 0b11 = standalone
    uint16_t seq_count,
    uint16_t payload_len     // bytes AFTER primary header
)
{
    /* Packet ID */
    hdr->packet_id =
          ((CCSDS_VERSION & 0x7) << 13)
        | ((type & 0x1) << 12)
        | ((sec_hdr_flag & 0x1) << 11)
        | (apid & 0x07FF);

    /* Sequence Control */
    hdr->sequence_ctrl =
          ((seq_flags & 0x3) << 14)
        | (seq_count & 0x3FFF);

    /* Packet Length = (total bytes after primary header) - 1 */
    hdr->packet_length = payload_len - 1;
}

/* Build a full CCSDS packet */
uint16_t ccsds_build_packet(
    uint8_t *out_buf,
    uint16_t out_buf_size,
    uint16_t apid,
    uint16_t seq_count,
    const uint8_t *payload,
    uint16_t payload_len
)
{
    if (out_buf_size < CCSDS_PRIMARY_HEADER_LEN + payload_len)
        return 0;

    ccsds_primary_header_t hdr;

    ccsds_build_primary_header(
        &hdr,
        CCSDS_TYPE_TM,   // Telemetry
        0,               // No secondary header
        apid,
        0x3,             // Standalone packet
        seq_count,
        payload_len
    );

    memcpy(out_buf, &hdr, CCSDS_PRIMARY_HEADER_LEN);
    memcpy(out_buf + CCSDS_PRIMARY_HEADER_LEN, payload, payload_len);

    return CCSDS_PRIMARY_HEADER_LEN + payload_len;
}
