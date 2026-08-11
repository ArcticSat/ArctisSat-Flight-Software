//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Functions for NSP SLIP protocol used by Rocket Lab ST16
//
// History
// 2026-08-06 Riley Sweeney
// - Created
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


#include "drivers/protocol/nsp_slip.h"
#include <string.h>

/* ---------------------------------------------------------------
   CRC-16 as specified in Section 6.3.7 (same polynomial, LSB first)
   --------------------------------------------------------------- */
#define POLY 0x8408  /* bits reversed for LSB first */

uint16_t nsp_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t ch = buf[i];
        for (int j = 0; j < 8; j++) {
            if ((ch ^ crc) & 0x01)
                crc = (crc >> 1) ^ POLY;
            else
                crc >>= 1;
            ch >>= 1;
        }
    }
    return crc;
}

/* ---------------------------------------------------------------
   SLIP decoder
   --------------------------------------------------------------- */
void nsp_slip_decoder_init(nsp_slip_decoder_t *dec)
{
    dec->idx = 0;
    dec->escape = false;
}

static void nsp_slip_decode_byte(nsp_slip_decoder_t *dec, uint8_t b,
                                 void (*callback)(const nsp_message_t *msg))
{
    if (b == SLIP_FEND) {
        /* End of frame: if we collected at least 5 bytes, check CRC */
        if (dec->idx >= 5) {
            uint16_t received_crc = (dec->buf[dec->idx - 1] << 8) |
                                    dec->buf[dec->idx - 2];
            uint16_t computed_crc = nsp_crc16(dec->buf, dec->idx - 2);
            if (received_crc == computed_crc) {
                /* Build a message struct and call callback */
                nsp_message_t msg;
                msg.dest    = dec->buf[0];
                msg.src     = dec->buf[1];
                msg.control = dec->buf[2];
                msg.data_len = dec->idx - 5;  /* bytes before CRC */
                if (msg.data_len <= NSP_MAX_DATA_APP) {
                    memcpy(msg.data, &dec->buf[3], msg.data_len);
                    callback(&msg);
                }
            }
        }
        /* Reset for next frame */
        dec->idx = 0;
        dec->escape = false;
        return;
    }

    if (dec->escape) {
        /* Translate escaped byte */
        if (b == SLIP_TFEND)      b = SLIP_FEND;
        else if (b == SLIP_TFESC) b = SLIP_FESC;
        dec->escape = false;
    } else if (b == SLIP_FESC) {
        dec->escape = true;
        return;
    }

    /* Store byte if room */
    if (dec->idx < sizeof(dec->buf)) {
        dec->buf[dec->idx++] = b;
    }
    /* else overflow – frame is too long, ignore until next FEND */
}

void nsp_slip_process_bytes(nsp_slip_decoder_t *dec,
                            const uint8_t *data, size_t len,
                            void (*callback)(const nsp_message_t *msg))
{
    for (size_t i = 0; i < len; i++) {
        nsp_slip_decode_byte(dec, data[i], callback);
    }
}

/* ---------------------------------------------------------------
   SLIP encoder & sender
   --------------------------------------------------------------- */
void nsp_slip_send(uint8_t dest, uint8_t src, uint8_t control,
                   const uint8_t *data, uint16_t data_len,
                   void (*send_func)(const uint8_t *buf, size_t len))
{
    /* Build plain NSP message in a local buffer */
    uint8_t raw[NSP_MAX_MSG_APP];
    size_t raw_len = 0;

    raw[raw_len++] = dest;
    raw[raw_len++] = src;
    raw[raw_len++] = control;
    if (data_len > NSP_MAX_DATA_APP) data_len = NSP_MAX_DATA_APP; // clip
    if (data_len > 0) {
        memcpy(&raw[raw_len], data, data_len);
        raw_len += data_len;
    }
    /* Append CRC (little endian) */
    uint16_t crc = nsp_crc16(raw, raw_len);
    raw[raw_len++] = crc & 0xFF;
    raw[raw_len++] = (crc >> 8) & 0xFF;

    /* SLIP encode and transmit in 256‑byte chunks (for typical UART drivers) */
    uint8_t tx_buf[2 * sizeof(raw) + 2];   // worst‑case expansion + two FENDs
    size_t tx_idx = 0;

    tx_buf[tx_idx++] = SLIP_FEND;
    for (size_t i = 0; i < raw_len; i++) {
        uint8_t b = raw[i];
        if (b == SLIP_FEND) {
            tx_buf[tx_idx++] = SLIP_FESC;
            tx_buf[tx_idx++] = SLIP_TFEND;
        } else if (b == SLIP_FESC) {
            tx_buf[tx_idx++] = SLIP_FESC;
            tx_buf[tx_idx++] = SLIP_TFESC;
        } else {
            tx_buf[tx_idx++] = b;
        }
    }
    tx_buf[tx_idx++] = SLIP_FEND;

    send_func(tx_buf, tx_idx);
}
