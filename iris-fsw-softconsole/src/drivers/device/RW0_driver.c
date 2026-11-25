//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Device driver for the reaction wheel controller using SPI with HDLC-like framing.
//  Implements CRC16-CCITT, HDLC framing/unframing, and all ICD commands.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "drivers/device/RW0_driver.h"
#include <string.h>
#include "drivers/protocol/spi.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// CRC16-CCITT TABLE
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// MODULE STATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static rw_config_t rw_cfg;
static bool rw_initialized = false;

// Internal buffers
static uint8_t tx_raw_buf[RW_MAX_FRAME_SIZE];
static uint8_t tx_frame_buf[RW_MAX_FRAME_SIZE];
static uint8_t rx_raw_buf[RW_MAX_FRAME_SIZE];
static uint8_t rx_frame_buf[RW_MAX_FRAME_SIZE];

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// CRC16-CCITT
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint16_t rw_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc = (crc << 8) ^ crc_table[((crc >> 8) ^ data[i]) & 0x00FF];
    }
    return crc;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// HDLC FRAMING: Stuff a payload into a frame with flag bytes and byte-stuffing.
// Returns the total frame length.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint16_t rw_frame_encode(const uint8_t *payload, uint16_t payload_len, uint8_t *frame_out)
{
    uint16_t idx = 0;

    // Start flag
    frame_out[idx++] = RW_FLAG_BYTE;

    // Payload with byte-stuffing
    for (uint16_t i = 0; i < payload_len; i++)
    {
        if (payload[i] == RW_FLAG_BYTE || payload[i] == RW_ESCAPE_BYTE)
        {
            frame_out[idx++] = RW_ESCAPE_BYTE;
            frame_out[idx++] = payload[i] ^ RW_ESCAPE_XOR;
        }
        else
        {
            frame_out[idx++] = payload[i];
        }
    }

    // End flag
    frame_out[idx++] = RW_FLAG_BYTE;

    return idx;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// HDLC UNFRAMING: Extract payload from a frame, removing flag bytes and unstuffing.
// Returns the payload length, or 0 if no valid frame found.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint16_t rw_frame_decode(const uint8_t *frame, uint16_t frame_len, uint8_t *payload_out)
{
    uint16_t payload_idx = 0;
    bool in_frame = false;
    bool escaped = false;

    for (uint16_t i = 0; i < frame_len; i++)
    {
        uint8_t byte = frame[i];

        if (!in_frame)
        {
            if (byte == RW_FLAG_BYTE)
            {
                in_frame = true;
                escaped = false;
            }
            continue;
        }

        // We are inside a frame
        if (byte == RW_FLAG_BYTE)
        {
            // End of frame (or empty frame)
            if (payload_idx > 0)
            {
                return payload_idx;  // Valid frame received
            }
            // Empty frame (consecutive flags) - reset and keep looking
            payload_idx = 0;
            escaped = false;
            continue;
        }

        if (byte == RW_ESCAPE_BYTE)
        {
            escaped = true;
            continue;
        }

        if (escaped)
        {
            payload_out[payload_idx++] = byte ^ RW_ESCAPE_XOR;
            escaped = false;
        }
        else
        {
            payload_out[payload_idx++] = byte;
        }
    }

    return 0;  // No complete frame found
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Build a request packet: [Command ID] [Request Data (optional)] [CRC16]
// Returns the payload length (before framing).
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint16_t rw_build_request(uint8_t cmd_id, const uint8_t *data, uint16_t data_len, uint8_t *payload_out)
{
    uint16_t idx = 0;

    payload_out[idx++] = cmd_id;

    if (data != NULL && data_len > 0)
    {
        memcpy(&payload_out[idx], data, data_len);
        idx += data_len;
    }

    // CRC over command + data
    uint16_t crc = rw_crc16(payload_out, idx);
    payload_out[idx++] = (uint8_t)(crc & 0xFF);         // CRC low byte (little endian)
    payload_out[idx++] = (uint8_t)((crc >> 8) & 0xFF);  // CRC high byte

    return idx;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Validate a reply packet: [Command ID] [Result Code] [Reply Data (optional)] [CRC16]
// Returns RW_OK if valid, error code otherwise.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static rw_error_t rw_validate_reply(const uint8_t *payload, uint16_t payload_len,
                                     uint8_t expected_cmd, uint8_t *result_code,
                                     const uint8_t **reply_data, uint16_t *reply_data_len)
{
    // Minimum reply: cmd(1) + result(1) + crc(2) = 4 bytes
    if (payload_len < 4)
    {
        return RW_ERR_INVALID_RESPONSE;
    }

    // Check command ID
    if (payload[0] != expected_cmd)
    {
        return RW_ERR_INVALID_RESPONSE;
    }

    // Extract result code
    *result_code = payload[1];

    // Verify CRC (over everything except the CRC bytes)
    uint16_t data_len = payload_len - 2;  // Exclude CRC bytes
    uint16_t crc_received = (uint16_t)payload[payload_len - 2] | ((uint16_t)payload[payload_len - 1] << 8);
    uint16_t crc_calculated = rw_crc16(payload, data_len);

    if (crc_received != crc_calculated)
    {
        return RW_ERR_CRC;
    }

    // Extract reply data (between result code and CRC)
    if (payload_len > 4)
    {
        *reply_data = &payload[2];
        *reply_data_len = payload_len - 4;
    }
    else
    {
        *reply_data = NULL;
        *reply_data_len = 0;
    }

    return RW_OK;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Send a framed packet over SPI.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static rw_error_t rw_spi_send(const uint8_t *frame, uint16_t frame_len)
{
    if (rw_cfg.use_gpio_ss)
    {
        // Use GPIO slave select (held low for entire transaction)
        SPI_GPIO_SS_ENABLE(rw_cfg.ss_gpio);
        spi_transaction_block_write_without_toggle(rw_cfg.spi_core, rw_cfg.spi_slave,
                                                    (uint8_t *)frame, frame_len, NULL, 0);
        SPI_GPIO_SS_DISABLE(rw_cfg.ss_gpio);
    }
    else
    {
        // Use SPI peripheral slave select
        spi_transaction_block_write_without_toggle(rw_cfg.spi_core, rw_cfg.spi_slave,
                                                    (uint8_t *)frame, frame_len, NULL, 0);
    }
    return RW_OK;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Send empty frames (flag bytes) and receive response from slave.
// The slave responds with empty frames until it has a reply ready.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static rw_error_t rw_spi_receive(uint8_t expected_cmd, uint8_t *result_code,
                                  uint8_t *reply_data, uint16_t *reply_data_len,
                                  uint32_t max_attempts)
{
    // Send empty frames (0x7E) to clock out the response
    uint8_t empty_frame[RW_MAX_FRAME_SIZE];
    memset(empty_frame, RW_FLAG_BYTE, RW_MAX_FRAME_SIZE);

    for (uint32_t attempt = 0; attempt < max_attempts; attempt++)
    {
        memset(rx_raw_buf, 0x00, RW_MAX_FRAME_SIZE);

        if (rw_cfg.use_gpio_ss)
        {
            SPI_GPIO_SS_ENABLE(rw_cfg.ss_gpio);
            spi_transaction_block_read_without_toggle(rw_cfg.spi_core, rw_cfg.spi_slave,
                                                       empty_frame, RW_MAX_FRAME_SIZE,
                                                       rx_raw_buf, RW_MAX_FRAME_SIZE);
            SPI_GPIO_SS_DISABLE(rw_cfg.ss_gpio);
        }
        else
        {
            spi_transaction_block_read_without_toggle(rw_cfg.spi_core, rw_cfg.spi_slave,
                                                       empty_frame, RW_MAX_FRAME_SIZE,
                                                       rx_raw_buf, RW_MAX_FRAME_SIZE);
        }

        // Try to decode a frame from the received data
        uint16_t payload_len = rw_frame_decode(rx_raw_buf, RW_MAX_FRAME_SIZE, rx_frame_buf);

        if (payload_len > 0)
        {
            // Validate the reply
            const uint8_t *data_ptr = NULL;
            uint16_t data_len = 0;
            rw_error_t err = rw_validate_reply(rx_frame_buf, payload_len, expected_cmd,
                                                result_code, &data_ptr, &data_len);
            if (err == RW_OK)
            {
                if (reply_data != NULL && data_ptr != NULL && data_len > 0)
                {
                    memcpy(reply_data, data_ptr, data_len);
                }
                if (reply_data_len != NULL)
                {
                    *reply_data_len = data_len;
                }
                return RW_OK;
            }
            return err;
        }
        // No valid frame yet, keep polling
    }

    return RW_ERR_TIMEOUT;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Send a command that expects no reply.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static rw_error_t rw_send_no_reply(uint8_t cmd_id, const uint8_t *data, uint16_t data_len)
{
    // Build request payload
    uint16_t payload_len = rw_build_request(cmd_id, data, data_len, tx_raw_buf);

    // Frame it
    uint16_t frame_len = rw_frame_encode(tx_raw_buf, payload_len, tx_frame_buf);

    // Send
    return rw_spi_send(tx_frame_buf, frame_len);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Send a command that expects a reply.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static rw_error_t rw_send_with_reply(uint8_t cmd_id, const uint8_t *data, uint16_t data_len,
                                      uint8_t *result_code, uint8_t *reply_data,
                                      uint16_t *reply_data_len)
{
    // Build request payload
    uint16_t payload_len = rw_build_request(cmd_id, data, data_len, tx_raw_buf);

    // Frame it
    uint16_t frame_len = rw_frame_encode(tx_raw_buf, payload_len, tx_frame_buf);

    // Send request
    rw_error_t err = rw_spi_send(tx_frame_buf, frame_len);
    if (err != RW_OK)
    {
        return err;
    }

    // Receive reply
    return rw_spi_receive(cmd_id, result_code, reply_data, reply_data_len, RW_MAX_POLL_ATTEMPTS);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// PUBLIC API
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

rw_error_t rw_init(const rw_config_t *config)
{
    if (config == NULL)
    {
        return RW_ERR_INVALID_PARAM;
    }

    memcpy(&rw_cfg, config, sizeof(rw_config_t));

    // Configure GPIO slave select if used
    if (rw_cfg.use_gpio_ss)
    {
        spi_configure_gpio_ss(rw_cfg.ss_gpio);
    }

    rw_initialized = true;
    return RW_OK;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_reset_mcu(void)
{
    if (!rw_initialized) return RW_ERR_INVALID_PARAM;
    return rw_send_no_reply(RW_CMD_RESET_MCU, NULL, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_last_reset_status(uint8_t *status)
{
    if (!rw_initialized || status == NULL) return RW_ERR_INVALID_PARAM;

    uint8_t result_code = 0;
    uint8_t reply_data[4] = {0};
    uint16_t reply_len = 0;

    rw_error_t err = rw_send_with_reply(RW_CMD_GET_LAST_RESET_STATUS, NULL, 0,
                                         &result_code, reply_data, &reply_len);
    if (err != RW_OK) return err;
    if (result_code != RW_RESULT_OK) return RW_ERR_INVALID_RESPONSE;
    if (reply_len < 1) return RW_ERR_INVALID_RESPONSE;

    *status = reply_data[0];
    return RW_OK;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_clear_last_reset_status(void)
{
    if (!rw_initialized) return RW_ERR_INVALID_PARAM;
    return rw_send_no_reply(RW_CMD_CLEAR_LAST_RESET_STATUS, NULL, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_status(rw_status_t *status)
{
    if (!rw_initialized || status == NULL) return RW_ERR_INVALID_PARAM;

    uint8_t result_code = 0;
    uint8_t reply_data[16] = {0};
    uint16_t reply_len = 0;

    rw_error_t err = rw_send_with_reply(RW_CMD_GET_RW_STATUS, NULL, 0,
                                         &result_code, reply_data, &reply_len);
    if (err != RW_OK) return err;
    if (result_code != RW_RESULT_OK) return RW_ERR_INVALID_RESPONSE;
    if (reply_len < 10) return RW_ERR_INVALID_RESPONSE;

    // Parse little-endian data
    uint16_t idx = 0;
    memcpy(&status->current_speed, &reply_data[idx], 4);   idx += 4;
    memcpy(&status->reference_speed, &reply_data[idx], 4); idx += 4;
    status->state = reply_data[idx++];
    status->clc_mode = reply_data[idx++];

    return RW_OK;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_initialize_controller(void)
{
    if (!rw_initialized) return RW_ERR_INVALID_PARAM;
    return rw_send_no_reply(RW_CMD_INIT_RW_CONTROLLER, NULL, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_set_reference_speed(int32_t speed, uint16_t ramp_time)
{
    if (!rw_initialized) return RW_ERR_INVALID_PARAM;

    // Validate speed
    if (speed > 0 && (speed < RW_SPEED_CW_MIN || speed > RW_SPEED_CW_MAX))
    {
        return RW_ERR_INVALID_PARAM;
    }
    if (speed < 0 && (speed < RW_SPEED_CCW_MIN || speed > RW_SPEED_CCW_MAX))
    {
        return RW_ERR_INVALID_PARAM;
    }
    if (speed == 0)
    {
        return RW_ERR_INVALID_PARAM;
    }

    // Validate ramp time
    if (ramp_time < RW_RAMP_TIME_MIN || ramp_time > RW_RAMP_TIME_MAX)
    {
        return RW_ERR_INVALID_PARAM;
    }

    // Build request data (little endian)
    uint8_t data[6];
    memcpy(&data[0], &speed, 4);
    memcpy(&data[4], &ramp_time, 2);

    return rw_send_no_reply(RW_CMD_SET_REFERENCE_SPEED, data, 6);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_set_current_limit_mode(rw_clc_mode_t mode)
{
    if (!rw_initialized) return RW_ERR_INVALID_PARAM;
    if (mode != RW_CLC_MODE_LOW_CURRENT && mode != RW_CLC_MODE_HIGH_CURRENT)
    {
        return RW_ERR_INVALID_PARAM;
    }

    uint8_t data = (uint8_t)mode;
    return rw_send_no_reply(RW_CMD_SET_CURRENT_LIMIT_MODE, &data, 1);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_temperature(int32_t *temperature)
{
    if (!rw_initialized || temperature == NULL) return RW_ERR_INVALID_PARAM;

    uint8_t result_code = 0;
    uint8_t reply_data[8] = {0};
    uint16_t reply_len = 0;

    rw_error_t err = rw_send_with_reply(RW_CMD_GET_TEMPERATURE, NULL, 0,
                                         &result_code, reply_data, &reply_len);
    if (err != RW_OK) return err;
    if (result_code != RW_RESULT_OK) return RW_ERR_INVALID_RESPONSE;
    if (reply_len < 4) return RW_ERR_INVALID_RESPONSE;

    memcpy(temperature, &reply_data[0], 4);
    return RW_OK;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_telemetry(rw_telemetry_t *telemetry)
{
    if (!rw_initialized || telemetry == NULL) return RW_ERR_INVALID_PARAM;

    uint8_t result_code = 0;
    uint8_t reply_data[RW_MAX_PAYLOAD_SIZE] = {0};
    uint16_t reply_len = 0;

    rw_error_t err = rw_send_with_reply(RW_CMD_GET_TELEMETRY, NULL, 0,
                                         &result_code, reply_data, &reply_len);
    if (err != RW_OK) return err;
    if (result_code != RW_RESULT_OK) return RW_ERR_INVALID_RESPONSE;
    if (reply_len < 87) return RW_ERR_INVALID_RESPONSE;

    // Parse little-endian telemetry data
    uint16_t idx = 0;
    telemetry->last_reset_status = reply_data[idx++];
    memcpy(&telemetry->mcu_temperature, &reply_data[idx], 4);              idx += 4;
    memcpy(&telemetry->pressure_sensor_temperature, &reply_data[idx], 4);  idx += 4;
    memcpy(&telemetry->pressure, &reply_data[idx], 4);                     idx += 4;
    telemetry->rw_state = reply_data[idx++];
    telemetry->rw_clc_mode = reply_data[idx++];
    memcpy(&telemetry->rw_curr_speed, &reply_data[idx], 4);                idx += 4;
    memcpy(&telemetry->rw_ref_speed, &reply_data[idx], 4);                 idx += 4;
    memcpy(&telemetry->num_of_invalid_crc_packets, &reply_data[idx], 4);   idx += 4;
    memcpy(&telemetry->num_of_invalid_len_packets, &reply_data[idx], 4);   idx += 4;
    memcpy(&telemetry->num_of_invalid_cmd_packets, &reply_data[idx], 4);   idx += 4;
    memcpy(&telemetry->num_of_cmd_executed_requests, &reply_data[idx], 4); idx += 4;
    memcpy(&telemetry->num_of_cmd_replies, &reply_data[idx], 4);           idx += 4;
    memcpy(&telemetry->uart_num_of_bytes_written, &reply_data[idx], 4);    idx += 4;
    memcpy(&telemetry->uart_num_of_bytes_read, &reply_data[idx], 4);       idx += 4;
    memcpy(&telemetry->uart_num_of_parity_errors, &reply_data[idx], 4);    idx += 4;
    memcpy(&telemetry->uart_num_of_noise_errors, &reply_data[idx], 4);     idx += 4;
    memcpy(&telemetry->uart_num_of_frame_errors, &reply_data[idx], 4);     idx += 4;
    memcpy(&telemetry->uart_num_of_register_overrun_errors, &reply_data[idx], 4); idx += 4;
    memcpy(&telemetry->uart_total_num_of_errors, &reply_data[idx], 4);     idx += 4;
    memcpy(&telemetry->spi_num_of_bytes_written, &reply_data[idx], 4);     idx += 4;
    memcpy(&telemetry->spi_num_of_bytes_read, &reply_data[idx], 4);        idx += 4;
    memcpy(&telemetry->spi_num_of_register_overrun_errors, &reply_data[idx], 4); idx += 4;
    memcpy(&telemetry->spi_total_num_of_errors, &reply_data[idx], 4);      idx += 4;

    return RW_OK;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_ping(void)
{
    if (!rw_initialized) return RW_ERR_INVALID_PARAM;
    return rw_send_no_reply(RW_CMD_PING, NULL, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_system_info(rw_system_info_t *info)
{
    if (!rw_initialized || info == NULL) return RW_ERR_INVALID_PARAM;

    uint8_t result_code = 0;
    uint8_t reply_data[24] = {0};
    uint16_t reply_len = 0;

    rw_error_t err = rw_send_with_reply(RW_CMD_GET_SYSTEM_INFO, NULL, 0,
                                         &result_code, reply_data, &reply_len);
    if (err != RW_OK) return err;
    if (result_code != RW_RESULT_OK) return RW_ERR_INVALID_RESPONSE;
    if (reply_len < 20) return RW_ERR_INVALID_RESPONSE;

    uint16_t idx = 0;
    memcpy(&info->version_major, &reply_data[idx], 4);         idx += 4;
    memcpy(&info->version_build_number, &reply_data[idx], 4);  idx += 4;
    memcpy(&info->uid1, &reply_data[idx], 4);                  idx += 4;
    memcpy(&info->uid2, &reply_data[idx], 4);                  idx += 4;
    memcpy(&info->uid3, &reply_data[idx], 4);                  idx += 4;

    return RW_OK;
}
