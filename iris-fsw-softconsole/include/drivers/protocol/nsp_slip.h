#ifndef NSP_SLIP_H
#define NSP_SLIP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Maximum data payload for the star tracker application process */
#define NSP_MAX_DATA_APP    512
/* Maximum total NSP message length (without SLIP overhead) */
#define NSP_MAX_MSG_APP     (5 + NSP_MAX_DATA_APP)  /* 517 bytes */

/* SLIP special characters per RFC 1055 */
#define SLIP_FEND           0xC0
#define SLIP_FESC           0xDB
#define SLIP_TFEND          0xDC
#define SLIP_TFESC          0xDD

/* NSP message structure (received or to be sent) */
typedef struct {
    uint8_t dest;               /* Destination address */
    uint8_t src;                /* Source address */
    uint8_t control;            /* Command code & flags */
    uint8_t data[NSP_MAX_DATA_APP]; /* Data payload */
    uint16_t data_len;          /* Length of data in bytes */
} nsp_message_t;

/* NSP control byte bit definitions */
#define NSP_CTRL_PF             (1 << 7)   /* Poll/Final */
#define NSP_CTRL_ACK            (1 << 6)   /* Acknowledge (reply) */
#define NSP_CTRL_B              (1 << 5)   /* Copied from command to reply */
#define NSP_CTRL_CMD_MASK       0x1F        /* Command code bits */

/* ----- SLIP decoder state (use one instance per UART) ----- */
typedef struct {
    uint8_t buf[NSP_MAX_MSG_APP];
    size_t  idx;
    bool    escape;   /* true if previous byte was FESC */
} nsp_slip_decoder_t;

/* ---- API ---- */

/* Initialise SLIP decoder */
void nsp_slip_decoder_init(nsp_slip_decoder_t *dec);

/* Feed a chunk of received bytes into the decoder.
   When a complete, valid NSP message is assembled, callback(msg) is called.
   The message buffer is only valid during the callback; copy if needed. */
void nsp_slip_process_bytes(nsp_slip_decoder_t *dec,
                            const uint8_t *data, size_t len,
                            void (*callback)(const nsp_message_t *msg));

/* Build a complete NSP message and send it as a SLIP‑encoded frame.
   send_func is a hardware‑specific function that transmits a byte buffer. */
void nsp_slip_send(uint8_t dest, uint8_t src, uint8_t control,
                   const uint8_t *data, uint16_t data_len,
                   void (*send_func)(const uint8_t *buf, size_t len));

/* Compute NSP 16‑bit CRC (CCITT polynomial, LSB first) */
uint16_t nsp_crc16(const uint8_t *buf, size_t len);

#endif /* NSP_SLIP_H */
