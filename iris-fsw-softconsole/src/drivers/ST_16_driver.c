#include "drivers/ST_16_driver.h"

#include <string.h>

/* ------- Static module variables ------- */
static void (*hw_send)(const uint8_t *buf, size_t len) = NULL;
static st_driver_t *active_driver = NULL;

/* Forward declaration of internal callback */
static void on_nsp_message(const nsp_message_t *msg);

/* ------- Initialisation ------- */
void st_init(st_driver_t *drv,
             uint8_t my_addr,
             uint8_t supervisor_addr,
             uint8_t functional_addr,
             void (*send_func)(const uint8_t *buf, size_t len),
             void (*reply_callback)(const nsp_message_t *msg))
{
    drv->my_nsp_addr         = my_addr;
    drv->st_supervisor_addr  = supervisor_addr;
    drv->st_functional_addr  = functional_addr;
    drv->reply_callback      = reply_callback;
    nsp_slip_decoder_init(&drv->slip_dec);
    hw_send = send_func;
    active_driver = drv;
}

void st_process_rx_bytes(st_driver_t *drv, const uint8_t *data, size_t len)
{
    nsp_slip_process_bytes(&drv->slip_dec, data, len, on_nsp_message);
}

/* ------- Generic internal command builder ------- */
static void send_nsp(uint8_t dest, uint8_t src, uint8_t control,
                     const uint8_t *data, uint16_t len)
{
    if (hw_send)
        nsp_slip_send(dest, src, control, data, len, hw_send);
}

void st_send_command(st_driver_t *drv,
                     uint8_t dest, st_command_code_t cmd,
                     const uint8_t *data, uint16_t data_len,
                     bool poll)
{
    uint8_t control = (uint8_t)(cmd & NSP_CTRL_CMD_MASK);
    if (poll)
        control |= NSP_CTRL_PF;
    send_nsp(dest, drv->my_nsp_addr, control, data, data_len);
}

/* ------- Callback from SLIP decoder ------- */
static void on_nsp_message(const nsp_message_t *msg)
{
    if (active_driver && active_driver->reply_callback)
        active_driver->reply_callback(msg);
}

/* ------- Convenience wrappers ------- */
void st_ping(st_driver_t *drv, uint8_t dest)
{
    st_send_command(drv, dest, ST_CMD_PING, NULL, 0, true);
}

void st_init_reboot(st_driver_t *drv, uint8_t dest)
{
    st_send_command(drv, dest, ST_CMD_INIT, NULL, 0, true);
}

void st_init_run(st_driver_t *drv, uint8_t dest, uint32_t address)
{
    uint8_t data[4];
    data[0] = address & 0xFF;
    data[1] = (address >> 8) & 0xFF;
    data[2] = (address >> 16) & 0xFF;
    data[3] = (address >> 24) & 0xFF;
    st_send_command(drv, dest, ST_CMD_INIT, data, 4, true);
}

void st_go(st_driver_t *drv, uint8_t go_code)
{
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_GO, &go_code, 1, true);
}

void st_read_result_short(st_driver_t *drv, uint16_t result_addr, uint8_t len)
{
    uint8_t data[3];
    data[0] = result_addr & 0xFF;
    data[1] = (result_addr >> 8) & 0xFF;
    data[2] = len;   // 0 = 256
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_READ_RESULT, data, 3, true);
}

void st_read_result_long(st_driver_t *drv, uint16_t result_addr, uint16_t len)
{
    uint8_t data[4];
    data[0] = result_addr & 0xFF;
    data[1] = (result_addr >> 8) & 0xFF;
    data[2] = len & 0xFF;
    data[3] = (len >> 8) & 0xFF;
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_READ_RESULT, data, 4, true);
}

void st_read_edac_short(st_driver_t *drv, uint16_t edac_addr, uint8_t len)
{
    uint8_t data[3];
    data[0] = edac_addr & 0xFF;
    data[1] = (edac_addr >> 8) & 0xFF;
    data[2] = len;
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_READ_EDAC, data, 3, true);
}

void st_read_edac_long(st_driver_t *drv, uint16_t edac_addr, uint16_t len)
{
    uint8_t data[4];
    data[0] = edac_addr & 0xFF;
    data[1] = (edac_addr >> 8) & 0xFF;
    data[2] = len & 0xFF;
    data[3] = (len >> 8) & 0xFF;
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_READ_EDAC, data, 4, true);
}

void st_write_edac(st_driver_t *drv, uint16_t edac_addr, const uint8_t *wdata, uint16_t len)
{
    /* Command data = address (2 bytes) + payload */
    uint8_t data[2 + len];
    data[0] = edac_addr & 0xFF;
    data[1] = (edac_addr >> 8) & 0xFF;
    memcpy(&data[2], wdata, len);
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_WRITE_EDAC, data, 2 + len, true);
}

void st_combination(st_driver_t *drv, uint8_t go_code, uint32_t bitmap)
{
    uint8_t data[4];
    data[0] = go_code;
    data[1] = bitmap & 0xFF;
    data[2] = (bitmap >> 8) & 0xFF;
    data[3] = (bitmap >> 16) & 0xFF;
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_COMBINATION, data, 4, true);
    /* Note: reply may be multiple messages; use reply callback to collect */
}

void st_read_time(st_driver_t *drv)
{
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_READ_TIME, NULL, 0, true);
}

void st_write_time(st_driver_t *drv, const uint8_t time_us_since_j2000[7])
{
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_WRITE_TIME,
                    time_us_since_j2000, 7, true);
}

void st_write_keps(st_driver_t *drv,
                   float e, float i, float raan, float argp,
                   float mean_motion, const uint8_t time_perigee[7])
{
    uint8_t data[26];
    /* Floats in little-endian */
    memcpy(&data[0],  &e, 4);
    memcpy(&data[4],  &i, 4);
    memcpy(&data[8],  &raan, 4);
    memcpy(&data[12], &argp, 4);
    memcpy(&data[16], &mean_motion, 4);
    memcpy(&data[20], time_perigee, 7);  /* 56-bit time */
    st_send_command(drv, drv->st_supervisor_addr, ST_CMD_WRITE_KEPS, data, 27, true);
}
