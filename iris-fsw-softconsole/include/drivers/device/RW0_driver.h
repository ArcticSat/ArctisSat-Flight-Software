#ifndef INCLUDE_REACTION_WHEEL_H_
#define INCLUDE_REACTION_WHEEL_H_

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Device driver for the reaction wheel controller using SPI with HDLC-like framing.
//  Implements the protocol defined in the Software ICD Notes.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "drivers/protocol/spi.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define RW_FLAG_BYTE            0x7E
#define RW_ESCAPE_BYTE          0x7D
#define RW_ESCAPE_XOR           0x20

#define RW_MAX_FRAME_SIZE       128
#define RW_MAX_PAYLOAD_SIZE     100

// Command IDs
#define RW_CMD_RESET_MCU                    1
#define RW_CMD_GET_LAST_RESET_STATUS        2
#define RW_CMD_CLEAR_LAST_RESET_STATUS      3
#define RW_CMD_GET_RW_STATUS                4
#define RW_CMD_INIT_RW_CONTROLLER           5
#define RW_CMD_SET_REFERENCE_SPEED          6
#define RW_CMD_SET_CURRENT_LIMIT_MODE       7
#define RW_CMD_GET_TEMPERATURE              8
#define RW_CMD_GET_TELEMETRY                9
#define RW_CMD_PING                         10
#define RW_CMD_GET_SYSTEM_INFO              11

// Result codes
#define RW_RESULT_OK                        0x00
#define RW_RESULT_ERROR                     0x01

// Speed limits (dRPM)
#define RW_SPEED_CW_MIN                     1000
#define RW_SPEED_CW_MAX                     65000
#define RW_SPEED_CCW_MIN                    (-65000)
#define RW_SPEED_CCW_MAX                    (-1000)

// Ramp time limits (ms)
#define RW_RAMP_TIME_MIN                    10
#define RW_RAMP_TIME_MAX                    20000

// Maximum number of empty frames to send while waiting for reply
#define RW_MAX_POLL_ATTEMPTS                50

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum {
    RW_STATE_ERROR          = 0,
    RW_STATE_IDLE           = 1,
    RW_STATE_COASTING       = 2,
    RW_STATE_RUNNING_STABLE = 3,
    RW_STATE_RUNNING_CHANGING = 4,
} rw_state_t;

typedef enum {
    RW_CLC_MODE_LOW_CURRENT  = 0,  // 0.3 A
    RW_CLC_MODE_HIGH_CURRENT = 1,  // 0.6 A (default)
} rw_clc_mode_t;

typedef enum {
    RW_OK = 0,
    RW_ERR_TIMEOUT,
    RW_ERR_CRC,
    RW_ERR_INVALID_RESPONSE,
    RW_ERR_INVALID_PARAM,
    RW_ERR_NO_REPLY_EXPECTED,
    RW_ERR_SPI,
} rw_error_t;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct {
    int32_t  current_speed;       // dRPM
    int32_t  reference_speed;     // dRPM
    uint8_t  state;               // rw_state_t
    uint8_t  clc_mode;            // rw_clc_mode_t
} rw_status_t;

typedef struct {
    uint8_t  last_reset_status;
    int32_t  mcu_temperature;
    float    pressure_sensor_temperature;
    float    pressure;
    uint8_t  rw_state;
    uint8_t  rw_clc_mode;
    int32_t  rw_curr_speed;
    int32_t  rw_ref_speed;
    uint32_t num_of_invalid_crc_packets;
    uint32_t num_of_invalid_len_packets;
    uint32_t num_of_invalid_cmd_packets;
    uint32_t num_of_cmd_executed_requests;
    uint32_t num_of_cmd_replies;
    uint32_t uart_num_of_bytes_written;
    uint32_t uart_num_of_bytes_read;
    uint32_t uart_num_of_parity_errors;
    uint32_t uart_num_of_noise_errors;
    uint32_t uart_num_of_frame_errors;
    uint32_t uart_num_of_register_overrun_errors;
    uint32_t uart_total_num_of_errors;
    uint32_t spi_num_of_bytes_written;
    uint32_t spi_num_of_bytes_read;
    uint32_t spi_num_of_register_overrun_errors;
    uint32_t spi_total_num_of_errors;
} rw_telemetry_t;

typedef struct {
    uint32_t version_major;
    uint32_t version_build_number;
    uint32_t uid1;
    uint32_t uid2;
    uint32_t uid3;
} rw_system_info_t;

typedef struct {
    CoreSPIInstance_t spi_core;
    spi_slave_t       spi_slave;
    mss_gpio_id_t     ss_gpio;       // GPIO pin for slave select (if using GPIO SS)
    bool              use_gpio_ss;
} rw_config_t;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Initialize the reaction wheel driver.
// Returns RW_OK on success.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_init(const rw_config_t *config);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 1: Reset MCU (no reply expected).
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_reset_mcu(void);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 2: Get last reset status.
// Returns the reset status byte via the pointer.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_last_reset_status(uint8_t *status);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 3: Clear last reset status (no reply expected).
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_clear_last_reset_status(void);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 4: Get reaction wheel status.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_status(rw_status_t *status);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 5: Initialize reaction wheel controller (no reply expected).
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_initialize_controller(void);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 6: Set reference speed.
//   speed: [1000..65000] or [-65000..-1000] dRPM
//   ramp_time: [10..20000] ms
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_set_reference_speed(int32_t speed, uint16_t ramp_time);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 7: Set current limit control mode.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_set_current_limit_mode(rw_clc_mode_t mode);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 8: Get temperature.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_temperature(int32_t *temperature);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 9: Get telemetry.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_telemetry(rw_telemetry_t *telemetry);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 10: Ping (no reply expected).
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_ping(void);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Command 11: Get system information.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
rw_error_t rw_get_system_info(rw_system_info_t *info);

#endif /* INCLUDE_REACTION_WHEEL_H_ */
