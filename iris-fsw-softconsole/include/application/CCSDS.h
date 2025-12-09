#ifndef CCSDS_H
#define CCSDS_H

#include <stdint.h>

/* ================================
 * CCSDS Constants
 * ================================ */

#define CCSDS_PRIMARY_HEADER_LEN  6

/* CCSDS Version Number (always 0) */
#define CCSDS_VERSION             0x0

/* Packet Types */
#define CCSDS_TYPE_TM             0x0
#define CCSDS_TYPE_TC             0x1

/* Sequence Flags */
#define CCSDS_SEQ_CONTINUATION    0x0
#define CCSDS_SEQ_FIRST           0x1
#define CCSDS_SEQ_LAST            0x2
#define CCSDS_SEQ_STANDALONE      0x3

/* ================================
 * CCSDS Primary Header Structure
 * ================================ */
typedef struct
{
    uint16_t packet_id;
    uint16_t sequence_ctrl;
    uint16_t packet_length;
} ccsds_primary_header_t;

/* ================================
 * Function Prototypes
 * ================================ */

/**
 * @brief Build a CCSDS primary header
 *
 * @param hdr            Pointer to header struct to fill
 * @param type           CCSDS_TYPE_TM or CCSDS_TYPE_TC
 * @param sec_hdr_flag   1 if secondary header present, 0 otherwise
 * @param apid           Application Process ID (11 bits)
 * @param seq_flags      Sequence flags (CCSDS_SEQ_*)
 * @param seq_count      Sequence count (14 bits)
 * @param payload_len    Bytes following the primary header
 */
void ccsds_build_primary_header(
    ccsds_primary_header_t *hdr,
    uint8_t  type,
    uint8_t  sec_hdr_flag,
    uint16_t apid,
    uint8_t  seq_flags,
    uint16_t seq_count,
    uint16_t payload_len
);

/**
 * @brief Build a complete CCSDS packet (primary header + payload)
 *
 * @param out_buf        Output buffer
 * @param out_buf_size   Size of output buffer in bytes
 * @param apid           Application Process ID
 * @param seq_count      Sequence count
 * @param payload        Payload data pointer
 * @param payload_len    Payload length in bytes
 *
 * @return Total packet length in bytes, or 0 on error
 */
uint16_t ccsds_build_packet(
    uint8_t *out_buf,
    uint16_t out_buf_size,
    uint16_t apid,
    uint16_t seq_count,
    const uint8_t *payload,
    uint16_t payload_len
);



#endif /* CCSDS_H */
