#ifndef STAR_TRACKER_DRIVER_H
#define STAR_TRACKER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol/nsp_slip.h"

/* ------ Default addresses (can be changed) ------ */
#define ST_SUPERVISOR_ADDR_DEFAULT    0x0A   /* Half duplex RS485-0, see Table 9 */
#define ST_FUNCTIONAL_ADDR_DEFAULT    0x0B   /* Derived from supervisor, Table 10 */

/* ------ Command codes (Table 14) ------ */
typedef enum {
    ST_CMD_PING         = 0x00,
    ST_CMD_INIT         = 0x01,
    ST_CMD_PEEK         = 0x02,
    ST_CMD_POKE         = 0x03,
    ST_CMD_DIAGNOSTIC   = 0x04,
    ST_CMD_STORE        = 0x05,
    ST_CMD_CRC          = 0x06,   /* supervisor only */
    ST_CMD_FLASH        = 0x06,   /* functional only */
    ST_CMD_READ_FILE    = 0x07,
    ST_CMD_WRITE_FILE   = 0x08,
    ST_CMD_READ_EDAC    = 0x09,
    ST_CMD_WRITE_EDAC   = 0x0A,
    ST_CMD_GO           = 0x0B,
    ST_CMD_GATHER_RESULT= 0x0C,
    ST_CMD_READ_RESULT  = 0x0D,
    ST_CMD_IMAGE        = 0x10,
    ST_CMD_COMBINATION  = 0x12,
    ST_CMD_READ_TIME    = 0x13,
    ST_CMD_WRITE_TIME   = 0x14,
    ST_CMD_WRITE_KEPS   = 0x15
} st_command_code_t;

/* ------ Context structure ------ */
typedef struct {
    nsp_slip_decoder_t slip_dec;
    uint8_t my_nsp_addr;              /* our address (e.g., flight computer) */
    uint8_t st_supervisor_addr;
    uint8_t st_functional_addr;
    /* Callback when a reply message is received (optional) */
    void (*reply_callback)(const nsp_message_t *msg);
} st_driver_t;

/* ------ API ------ */

/* Initialise the driver. Set addresses and register callback.
   Send function must point to the UART TX routine (e.g., prvUARTSend wrapper). */
void st_init(st_driver_t *drv,
             uint8_t my_addr,
             uint8_t supervisor_addr,
             uint8_t functional_addr,
             void (*send_func)(const uint8_t *buf, size_t len),
             void (*reply_callback)(const nsp_message_t *msg));

/* Feed received bytes into the driver's SLIP decoder */
void st_process_rx_bytes(st_driver_t *drv, const uint8_t *data, size_t len);

/* Generic command sender (poll = true -> expects reply, final flag) */
void st_send_command(st_driver_t *drv,
                     uint8_t dest, st_command_code_t cmd,
                     const uint8_t *data, uint16_t data_len,
                     bool poll);

/* ------ Convenience command wrappers ------ */

/* PING – returns human-readable string */
void st_ping(st_driver_t *drv, uint8_t dest);

/* INIT – reboot (no data) or run program at address (4 bytes LE) */
void st_init_reboot(st_driver_t *drv, uint8_t dest);
void st_init_run(st_driver_t *drv, uint8_t dest, uint32_t address);

/* GO – transition states (GO code byte) */
void st_go(st_driver_t *drv, uint8_t go_code);

/* READ RESULT – short (result_addr, len) or long (result_addr, 16-bit len) */
void st_read_result_short(st_driver_t *drv, uint16_t result_addr, uint8_t len);
void st_read_result_long (st_driver_t *drv, uint16_t result_addr, uint16_t len);

/* READ EDAC */
void st_read_edac_short(st_driver_t *drv, uint16_t edac_addr, uint8_t len);
void st_read_edac_long (st_driver_t *drv, uint16_t edac_addr, uint16_t len);

/* WRITE EDAC */
void st_write_edac(st_driver_t *drv, uint16_t edac_addr, const uint8_t *data, uint16_t len);

/* COMBINATION (one-shot GO + result) */
void st_combination(st_driver_t *drv, uint8_t go_code, uint32_t bitmap);

/* READ TIME / WRITE TIME */
void st_read_time(st_driver_t *drv);
void st_write_time(st_driver_t *drv, const uint8_t time_us_since_j2000[7]);

/* WRITE KEPS (6 floats + 7-byte time) – see Section 6.5.20 */
void st_write_keps(st_driver_t *drv,
                   float e, float i, float raan, float argp,
                   float mean_motion, const uint8_t time_perigee[7]);

#endif /* STAR_TRACKER_DRIVER_H */
