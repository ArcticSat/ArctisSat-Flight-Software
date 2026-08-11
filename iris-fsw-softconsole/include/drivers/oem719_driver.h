#ifndef OEM719_DRIVER_H
#define OEM719_DRIVER_H

#include <stdint.h>
#include <stddef.h>

/**
 * @file oem719_driver.h
 * @brief NovAtel OEM719 GNSS Receiver CAN Bus Driver
 *
 * Driver for NovAtel OEM719 GNSS receiver communicating over CAN bus.
 * Supports: Position, Velocity, Time, Fix Quality data
 * Constellations: GPS, Galileo
 */

/* ===== CAN Message IDs ===== */
#define OEM719_BESTPOS_MSG_ID    0x123  /**< Best Position message ID */
#define OEM719_BESTVEL_MSG_ID    0x124  /**< Best Velocity message ID */
#define OEM719_TIME_MSG_ID       0x125  /**< Time message ID */
#define OEM719_CONFIG_MSG_ID     0x200  /**< Configuration command ID */
#define OEM719_ACK_MSG_ID        0x300  /**< Acknowledgment message ID */

/* ===== Message Indexes ===== */
#define OEM719_BESTPOS_IDX  1
#define OEM719_BESTVEL_IDX  2
#define OEM719_TIME_IDX     3

/* ===== Fix Status Enumerations ===== */
typedef enum {
    FIX_STATUS_SOLUTION_COMPUTED = 0,
    FIX_STATUS_INSUFFICIENT_OBSERVATIONS = 1,
    FIX_STATUS_NO_CONVERGENCE = 2,
    FIX_STATUS_SINGULARITY = 3,
    FIX_STATUS_COV_TRACE_EXCEEDS_MAX = 4,
    FIX_STATUS_TEST_DISTANCE_EXCEEDS_MAX = 5,
    FIX_STATUS_VELOCITY_EXCEEDS_MAX = 7,
    FIX_STATUS_VARIANCE_EXCEEDS_MAX = 8,
    FIX_STATUS_RESIDUALS_EXCEEDS_MAX = 9,
    FIX_STATUS_DELTA_POS_EXCEEDS_MAX = 10,
    FIX_STATUS_NEGATIVE_VAR = 11,
    FIX_STATUS_LARGE_RESIDUALS = 12,
    FIX_STATUS_RECEIVER_RANGE_CHECK_FAILURE = 26,
    FIX_STATUS_UNKNOWN_ERROR = 255,
} oem719_fix_status_t;

typedef enum {
    POS_TYPE_NONE = 0,
    POS_TYPE_FIXED = 1,
    POS_TYPE_FLOAT_RTK = 4,
    POS_TYPE_SBAS = 5,
    POS_TYPE_L1_FLOAT = 9,
    POS_TYPE_IONOSPHERE_FREE_LC = 10,
    POS_TYPE_NARROW_LANE = 11,
    POS_TYPE_SINGLE = 16,
    POS_TYPE_PSRDIFF = 17,
    POS_TYPE_WAAS = 18,
    POS_TYPE_PROPAGATED = 19,
    POS_TYPE_L1_INT = 20,
    POS_TYPE_WIDE_LANE = 21,
    POS_TYPE_L1_DGPS = 22,
    POS_TYPE_UNK = 255,
} oem719_pos_type_t;

/* ===== Data Structures ===== */

/**
 * BESTPOS message - Position information
 * Transmitted at configurable interval
 */
typedef struct {
    uint32_t week;           /**< GPS week number */
    uint32_t seconds;        /**< Seconds into week (ms resolution expected) */
    oem719_fix_status_t status; /**< Solution status */
    oem719_pos_type_t pos_type; /**< Position type */
    double latitude;         /**< Latitude (degrees) */
    double longitude;        /**< Longitude (degrees) */
    double height;           /**< Ellipsoidal height (meters) */
    float lat_std;           /**< Latitude standard deviation (meters) */
    float lon_std;           /**< Longitude standard deviation (meters) */
    float hgt_std;           /**< Height standard deviation (meters) */
    uint32_t num_satellites; /**< Number of satellites used */
    uint16_t reserved;       /**< Reserved field */
    float hdop;              /**< Horizontal dilution of precision */
} oem719_bestpos_t;

/**
 * BESTVEL message - Velocity information
 * Transmitted at configurable interval
 */
typedef struct {
    uint32_t week;           /**< GPS week number */
    uint32_t seconds;        /**< Seconds into week (ms resolution expected) */
    oem719_fix_status_t status; /**< Solution status */
    oem719_pos_type_t pos_type; /**< Position type */
    float horizontal_speed;  /**< Horizontal speed (m/s) */
    float vertical_speed;    /**< Vertical speed (m/s) */
    float track;             /**< Track over ground (degrees) */
    float speed_std;         /**< Speed standard deviation (m/s) */
    float track_std;         /**< Track standard deviation (degrees) */
    float vdop;              /**< Vertical dilution of precision */
} oem719_bestvel_t;

/**
 * TIME message - GPS time information
 * Contains UTC time, GPS time, and leap seconds
 */
typedef struct {
    uint32_t week;           /**< GPS week number */
    uint32_t seconds;        /**< Seconds into week (ms resolution expected) */
    int32_t utc_offset;      /**< UTC offset from GPS time (milliseconds) */
    uint32_t utc_year;       /**< Year (UTC) */
    uint32_t utc_month;      /**< Month (UTC) 1-12 */
    uint32_t utc_day;        /**< Day (UTC) 1-31 */
    uint32_t utc_hour;       /**< Hour (UTC) 0-23 */
    uint32_t utc_minute;     /**< Minute (UTC) 0-59 */
    uint32_t utc_second;     /**< Second (UTC) 0-59 */
    uint8_t leap_seconds;    /**< Current leap seconds */
} oem719_time_t;

/**
 * Complete GNSS solution containing all data
 */
typedef struct {
    oem719_bestpos_t position;
    oem719_bestvel_t velocity;
    oem719_time_t time;
    uint32_t data_valid_flags; /**< Bitmask: bit 0=pos, bit 1=vel, bit 2=time */
} oem719_gnss_solution_t;

/* ===== Configuration Commands ===== */

/**
 * Configuration actions for constellation selection
 */
typedef enum {
    OEM719_CONFIG_GPS_ENABLE = 0x01,
    OEM719_CONFIG_GALILEO_ENABLE = 0x02,
    OEM719_CONFIG_GLONASS_ENABLE = 0x04,
    OEM719_CONFIG_BEIDOU_ENABLE = 0x08,
} oem719_constellation_t;

/**
 * Configuration command structure
 */
typedef struct {
    uint8_t command_type;      /**< Type of configuration command */
    uint8_t constellation_mask;/**< Bitmask of enabled constellations */
    uint16_t message_rate_ms;  /**< Message rate in milliseconds */
} oem719_config_cmd_t;

/* ===== Driver Functions ===== */

/**
 * Initialize OEM719 driver and CAN interface
 * @param can_config Pointer to CAN configuration structure
 * @return 0 on success, negative on error
 */
int oem719_init(void);

/**
 * Send configuration command to receiver
 * @param config Configuration command
 * @return 0 on success, negative on error
 */
int oem719_send_config(const oem719_config_cmd_t *config);

/**
 * Enable specific GNSS constellations
 * @param constellation_mask Bitmask of constellations to enable
 * @return 0 on success, negative on error
 */
int oem719_set_constellations(uint8_t constellation_mask);

/**
 * Set message output rate
 * @param rate_ms Message rate in milliseconds
 * @return 0 on success, negative on error
 */
int oem719_set_message_rate(uint16_t rate_ms);

/**
 * Process incoming CAN message
 * @param can_id CAN message ID
 * @param data Message data bytes
 * @param dlc Data length code
 * @param solution Pointer to solution structure to update
 * @return Number of new messages received, 0 if incomplete
 */
int oem719_process_can_message(uint32_t can_id, const uint8_t *data,
                               uint8_t dlc, oem719_gnss_solution_t *solution);

/**
 * Check if solution data is valid
 * @param solution Pointer to solution structure
 * @return 1 if valid (all fields received), 0 if incomplete
 */
int oem719_is_solution_valid(const oem719_gnss_solution_t *solution);

/**
 * Get human-readable fix status string
 * @param status Fix status enum value
 * @return String description of status
 */
const char* oem719_get_fix_status_string(oem719_fix_status_t status);

/**
 * Get human-readable position type string
 * @param type Position type enum value
 * @return String description of type
 */
const char* oem719_get_pos_type_string(oem719_pos_type_t type);

#endif /* OEM719_DRIVER_H */
