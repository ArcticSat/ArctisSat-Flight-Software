#include "drivers/oem719_driver.h"
#include <string.h>
#include <math.h>

/**
 * @file oem719_driver.c
 * @brief NovAtel OEM719 GNSS Receiver CAN Bus Driver Implementation
 */

/* ===== Internal State Management ===== */

static struct {
    int initialized;
    uint32_t last_config_tick;
} oem719_state = {0};

/* ===== CAN Message Packing/Unpacking Utilities ===== */

/**
 * Extract uint32_t from little-endian byte array
 */
static uint32_t extract_uint32_le(const uint8_t *data, uint8_t offset) {
    return ((uint32_t)data[offset + 0] << 0)  |
           ((uint32_t)data[offset + 1] << 8)  |
           ((uint32_t)data[offset + 2] << 16) |
           ((uint32_t)data[offset + 3] << 24);
}

/**
 * Extract int32_t from little-endian byte array
 */
static int32_t extract_int32_le(const uint8_t *data, uint8_t offset) {
    return (int32_t)extract_uint32_le(data, offset);
}

/**
 * Extract uint16_t from little-endian byte array
 */
static uint16_t extract_uint16_le(const uint8_t *data, uint8_t offset) {
    return ((uint16_t)data[offset + 0] << 0) |
           ((uint16_t)data[offset + 1] << 8);
}

/**
 * Extract float from little-endian byte array (IEEE 754)
 */
static float extract_float_le(const uint8_t *data, uint8_t offset) {
    uint32_t val = extract_uint32_le(data, offset);
    return *(float*)&val;
}

/**
 * Extract double from little-endian byte array (IEEE 754)
 */
static double extract_double_le(const uint8_t *data, uint8_t offset) {
    uint32_t low = extract_uint32_le(data, offset);
    uint32_t high = extract_uint32_le(data, offset + 4);
    uint64_t val = ((uint64_t)high << 32) | low;
    return *(double*)&val;
}

/**
 * Pack uint32_t into little-endian byte array
 */
static void pack_uint32_le(uint8_t *data, uint8_t offset, uint32_t val) {
    data[offset + 0] = (uint8_t)(val >> 0);
    data[offset + 1] = (uint8_t)(val >> 8);
    data[offset + 2] = (uint8_t)(val >> 16);
    data[offset + 3] = (uint8_t)(val >> 24);
}

/**
 * Pack uint16_t into little-endian byte array
 */
static void pack_uint16_le(uint8_t *data, uint8_t offset, uint16_t val) {
    data[offset + 0] = (uint8_t)(val >> 0);
    data[offset + 1] = (uint8_t)(val >> 8);
}

/**
 * Pack float into little-endian byte array
 */
static void pack_float_le(uint8_t *data, uint8_t offset, float val) {
    uint32_t *ptr = (uint32_t*)&val;
    pack_uint32_le(data, offset, *ptr);
}

/* ===== Message Parsing Functions ===== */

/**
 * Parse BESTPOS message from CAN data (8-byte frame)
 * Note: OEM719 splits messages across multiple CAN frames
 */
static int parse_bestpos_frame(const uint8_t *data, uint8_t dlc,
                                oem719_bestpos_t *pos, uint8_t frame_num) {
    if (dlc < 8) return -1;

    /* Frame 0: Week, Status, Type */
    if (frame_num == 0) {
        pos->week = extract_uint32_le(data, 0);
        pos->status = (oem719_fix_status_t)data[4];
        pos->pos_type = (oem719_pos_type_t)data[5];
        return 0;
    }

    /* Frame 1: Latitude (8 bytes - double) */
    if (frame_num == 1) {
        pos->latitude = extract_double_le(data, 0);
        return 0;
    }

    /* Frame 2: Longitude (8 bytes - double) */
    if (frame_num == 2) {
        pos->longitude = extract_double_le(data, 0);
        return 0;
    }

    /* Frame 3: Height, then standards (8 bytes) */
    if (frame_num == 3) {
        pos->height = extract_double_le(data, 0);
        return 0;
    }

    /* Frame 4: Standard deviations and other fields */
    if (frame_num == 4) {
        pos->lat_std = extract_float_le(data, 0);
        pos->lon_std = extract_float_le(data, 4);
        return 0;
    }

    /* Frame 5: Height std, num satellites, HDOP */
    if (frame_num == 5) {
        pos->hgt_std = extract_float_le(data, 0);
        pos->num_satellites = extract_uint32_le(data, 4);
        return 0;
    }

    /* Frame 6: Reserved, HDOP */
    if (frame_num == 6) {
        pos->reserved = extract_uint16_le(data, 0);
        pos->hdop = extract_float_le(data, 2);
        return 1; /* Message complete */
    }

    return -1;
}

/**
 * Parse BESTVEL message from CAN data
 */
static int parse_bestvel_frame(const uint8_t *data, uint8_t dlc,
                                oem719_bestvel_t *vel, uint8_t frame_num) {
    if (dlc < 8) return -1;

    /* Frame 0: Week, Status, Type */
    if (frame_num == 0) {
        vel->week = extract_uint32_le(data, 0);
        vel->status = (oem719_fix_status_t)data[4];
        vel->pos_type = (oem719_pos_type_t)data[5];
        return 0;
    }

    /* Frame 1: Horizontal speed, Vertical speed */
    if (frame_num == 1) {
        vel->horizontal_speed = extract_float_le(data, 0);
        vel->vertical_speed = extract_float_le(data, 4);
        return 0;
    }

    /* Frame 2: Track, Speed std */
    if (frame_num == 2) {
        vel->track = extract_float_le(data, 0);
        vel->speed_std = extract_float_le(data, 4);
        return 0;
    }

    /* Frame 3: Track std, VDOP */
    if (frame_num == 3) {
        vel->track_std = extract_float_le(data, 0);
        vel->vdop = extract_float_le(data, 4);
        return 1; /* Message complete */
    }

    return -1;
}

/**
 * Parse TIME message from CAN data
 */
static int parse_time_frame(const uint8_t *data, uint8_t dlc,
                             oem719_time_t *time, uint8_t frame_num) {
    if (dlc < 8) return -1;

    /* Frame 0: Week, Seconds, UTC offset */
    if (frame_num == 0) {
        time->week = extract_uint32_le(data, 0);
        time->seconds = extract_uint32_le(data, 4);
        return 0;
    }

    /* Frame 1: UTC offset, Year, Month */
    if (frame_num == 1) {
        time->utc_offset = extract_int32_le(data, 0);
        time->utc_year = extract_uint16_le(data, 4);
        time->utc_month = data[6];
        time->utc_day = data[7];
        return 0;
    }

    /* Frame 2: Hour, Minute, Second, Leap seconds */
    if (frame_num == 2) {
        time->utc_hour = data[0];
        time->utc_minute = data[1];
        time->utc_second = data[2];
        time->leap_seconds = data[3];
        return 1; /* Message complete */
    }

    return -1;
}

/* ===== Public Driver Functions ===== */

int oem719_init(void) {
    if (oem719_state.initialized) {
        return 0;
    }

    oem719_state.initialized = 1;
    oem719_state.last_config_tick = 0;

    return 0;
}

int oem719_send_config(const oem719_config_cmd_t *config) {
    uint8_t data[8];

    if (!oem719_state.initialized) {
        return -1;
    }

    if (config == NULL) {
        return -1;
    }

    /* Format configuration message */
    memset(data, 0, sizeof(data));
    data[0] = config->command_type;
    data[1] = config->constellation_mask;
    pack_uint16_le(data, 2, config->message_rate_ms);

    /* Send via CAN bus */
    return can_send(OEM719_CONFIG_MSG_ID, data, 4);
}

int oem719_set_constellations(uint8_t constellation_mask) {
    oem719_config_cmd_t config;

    config.command_type = 0x01;
    config.constellation_mask = constellation_mask;
    config.message_rate_ms = 100;

    return oem719_send_config(&config);
}

int oem719_set_message_rate(uint16_t rate_ms) {
    oem719_config_cmd_t config;

    config.command_type = 0x02;
    config.constellation_mask = 0;
    config.message_rate_ms = rate_ms;

    return oem719_send_config(&config);
}

int oem719_process_can_message(uint32_t can_id, const uint8_t *data,
                               uint8_t dlc, oem719_gnss_solution_t *solution) {
    static uint8_t bestpos_frame_count = 0;
    static uint8_t bestvel_frame_count = 0;
    static uint8_t time_frame_count = 0;
    static oem719_bestpos_t pending_pos = {0};
    static oem719_bestvel_t pending_vel = {0};
    static oem719_time_t pending_time = {0};
    int message_complete = 0;

    if (solution == NULL || data == NULL) {
        return -1;
    }

    /* Process based on message ID */
    if (can_id == OEM719_BESTPOS_MSG_ID) {
        if (parse_bestpos_frame(data, dlc, &pending_pos, bestpos_frame_count) == 1) {
            solution->position = pending_pos;
            solution->data_valid_flags |= 0x01;
            bestpos_frame_count = 0;
            message_complete = 1;
        } else {
            bestpos_frame_count++;
        }
    }
    else if (can_id == OEM719_BESTVEL_MSG_ID) {
        if (parse_bestvel_frame(data, dlc, &pending_vel, bestvel_frame_count) == 1) {
            solution->velocity = pending_vel;
            solution->data_valid_flags |= 0x02;
            bestvel_frame_count = 0;
            message_complete = 1;
        } else {
            bestvel_frame_count++;
        }
    }
    else if (can_id == OEM719_TIME_MSG_ID) {
        if (parse_time_frame(data, dlc, &pending_time, time_frame_count) == 1) {
            solution->time = pending_time;
            solution->data_valid_flags |= 0x04;
            time_frame_count = 0;
            message_complete = 1;
        } else {
            time_frame_count++;
        }
    }
    else if (can_id == OEM719_ACK_MSG_ID) {
        /* Configuration acknowledgment - can be logged/handled here */
    }

    return message_complete;
}

int oem719_is_solution_valid(const oem719_gnss_solution_t *solution) {
    if (solution == NULL) {
        return 0;
    }

    /* Check if all three data types have been received */
    return (solution->data_valid_flags == 0x07) ? 1 : 0;
}

const char* oem719_get_fix_status_string(oem719_fix_status_t status) {
    switch (status) {
        case FIX_STATUS_SOLUTION_COMPUTED:
            return "Solution Computed";
        case FIX_STATUS_INSUFFICIENT_OBSERVATIONS:
            return "Insufficient Observations";
        case FIX_STATUS_NO_CONVERGENCE:
            return "No Convergence";
        case FIX_STATUS_SINGULARITY:
            return "Singularity";
        case FIX_STATUS_COV_TRACE_EXCEEDS_MAX:
            return "Covariance Trace Exceeds Max";
        case FIX_STATUS_TEST_DISTANCE_EXCEEDS_MAX:
            return "Test Distance Exceeds Max";
        case FIX_STATUS_VELOCITY_EXCEEDS_MAX:
            return "Velocity Exceeds Max";
        case FIX_STATUS_VARIANCE_EXCEEDS_MAX:
            return "Variance Exceeds Max";
        case FIX_STATUS_RESIDUALS_EXCEEDS_MAX:
            return "Residuals Exceed Max";
        case FIX_STATUS_DELTA_POS_EXCEEDS_MAX:
            return "Delta Position Exceeds Max";
        case FIX_STATUS_NEGATIVE_VAR:
            return "Negative Variance";
        case FIX_STATUS_LARGE_RESIDUALS:
            return "Large Residuals";
        case FIX_STATUS_RECEIVER_RANGE_CHECK_FAILURE:
            return "Receiver Range Check Failure";
        case FIX_STATUS_UNKNOWN_ERROR:
        default:
            return "Unknown Error";
    }
}

const char* oem719_get_pos_type_string(oem719_pos_type_t type) {
    switch (type) {
        case POS_TYPE_NONE:
            return "None";
        case POS_TYPE_FIXED:
            return "Fixed";
        case POS_TYPE_FLOAT_RTK:
            return "Float RTK";
        case POS_TYPE_SBAS:
            return "SBAS";
        case POS_TYPE_L1_FLOAT:
            return "L1 Float";
        case POS_TYPE_IONOSPHERE_FREE_LC:
            return "Ionosphere Free LC";
        case POS_TYPE_NARROW_LANE:
            return "Narrow Lane";
        case POS_TYPE_SINGLE:
            return "Single";
        case POS_TYPE_PSRDIFF:
            return "Pseudorange Differential";
        case POS_TYPE_WAAS:
            return "WAAS";
        case POS_TYPE_PROPAGATED:
            return "Propagated";
        case POS_TYPE_L1_INT:
            return "L1 Integer";
        case POS_TYPE_WIDE_LANE:
            return "Wide Lane";
        case POS_TYPE_L1_DGPS:
            return "L1 DGPS";
        case POS_TYPE_UNK:
        default:
            return "Unknown";
    }
}
