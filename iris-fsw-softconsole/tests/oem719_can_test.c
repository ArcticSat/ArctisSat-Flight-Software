#include "tests/oem719_can_test.h"

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/queue.h>
#include <FreeRTOS-Kernel/include/task.h>

#include <stdio.h>
#include <string.h>

/*
 * Test configuration.
 */
#ifndef OEM719_TEST_MESSAGE_RATE_MS
#define OEM719_TEST_MESSAGE_RATE_MS     100U
#endif

#ifndef OEM719_TEST_CONSTELLATION_MASK
#define OEM719_TEST_CONSTELLATION_MASK  (OEM719_CONFIG_GPS_ENABLE | OEM719_CONFIG_GALILEO_ENABLE)
#endif

#ifndef OEM719_TEST_QUEUE_WAIT_MS
#define OEM719_TEST_QUEUE_WAIT_MS       10U
#endif

#ifndef OEM719_TEST_CONFIG_RETRY_MS
#define OEM719_TEST_CONFIG_RETRY_MS     1000U
#endif

/*
 * Weak parser reset stub.
 *
 * Implement a real version in oem719_driver.c if you want to repeatedly run
 * the parser self-test or recover cleanly from missed frames.
 */
__attribute__((weak)) void oem719_reset_parser(void)
{
    /* Intentionally empty. */
}

/*
 * Weak CAN transmit glue function.
 *
 * oem719_driver.c expects:
 *
 *     int can_send(uint32_t id, const uint8_t *data, uint8_t dlc);
 *
 * current CAN layer exposes CAN_transmit_message(), so this connects
 * the OEM719 driver to CAN layer.
 *
 * If you already implemented can_send() elsewhere, delete this weak version
 * or let your strong implementation override it.
 */
__attribute__((weak)) int can_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    CANMessage_t msg;

    if ((data == NULL) || (dlc > MAX_CAN_DATA_LENGTH))
    {
        return -1;
    }

    memset(&msg, 0, sizeof(msg));

    msg.id  = id;
    msg.dlc = dlc;
    memcpy(msg.data, data, dlc);

    if (!CAN_TRANSMIT_READY())
    {
        return -1;
    }

    return (CAN_transmit_message(&msg) == 1) ? 0 : -1;
}

static int oem719_test_id_is_gnss(uint32_t id)
{
    return (id == OEM719_BESTPOS_MSG_ID) ||
           (id == OEM719_BESTVEL_MSG_ID) ||
           (id == OEM719_TIME_MSG_ID)     ||
           (id == OEM719_ACK_MSG_ID);
}

int oem719_can_comm_test(CANBaudRate baud,
                         QueueHandle_t rx_queue,
                         uint32_t timeout_ms,
                         oem719_test_result_t *result)
{
    if (result == NULL)
    {
        return -1;
    }

    memset(result, 0, sizeof(*result));

    /*
     * If caller did not provide a queue, use can_rx_queue.
     * Only call init_CAN() if can_rx_queue has not already been created.
     */
    if (rx_queue == NULL)
    {
        if (can_rx_queue == NULL)
        {
            /*
             * Current can.h prototype is:
             *
             *     int init_CAN(CANBaudRate baudrate,
             *                  QueueHandle_t *csp_rx_queue_handle);
             *
             * Passing NULL means we do not want to route CSP traffic from
             * this test. Make sure init_CAN() handles NULL safely.
             */
            if (init_CAN(baud, NULL) != 1)
            {
                result->can_init_ok = 0;
                return -1;
            }
        }

        result->can_init_ok = 1;
        rx_queue = can_rx_queue;
    }
    else
    {
        result->can_init_ok = 1;
    }

    if (rx_queue == NULL)
    {
        return -1;
    }

    if (oem719_init() != 0)
    {
        return -1;
    }

    result->driver_init_ok = 1;

    /*
     * Clean parser state if the driver provides a reset.
     */
    oem719_reset_parser();

    /*
     * Send basic configuration commands.
     *
     * These return 0 if the CAN controller accepted the frame. They do not
     * prove that the OEM719 acknowledged them unless an ACK frame is received.
     */
    if (oem719_set_message_rate(OEM719_TEST_MESSAGE_RATE_MS) == 0)
    {
        result->tx_config_ok = 1;
    }
    else
    {
        result->can_tx_error = 1;
    }

    if (oem719_set_constellations(OEM719_TEST_CONSTELLATION_MASK) == 0)
    {
        result->tx_constellation_ok = 1;
    }
    else
    {
        result->can_tx_error = 1;
    }

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    TickType_t last_config_tx = start_tick;

    const TickType_t config_retry_ticks = pdMS_TO_TICKS(OEM719_TEST_CONFIG_RETRY_MS);
    const TickType_t queue_wait_ticks = pdMS_TO_TICKS(OEM719_TEST_QUEUE_WAIT_MS);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        CANMessage_t msg;

        if (xQueueReceive(rx_queue, &msg, queue_wait_ticks) == pdTRUE)
        {
            result->rx_frames++;

            if (!oem719_test_id_is_gnss(msg.id))
            {
                result->non_oem_frames++;
                continue;
            }

            if (msg.id == OEM719_ACK_MSG_ID)
            {
                result->ack_received = 1;
            }

            int complete = oem719_process_can_message(msg.id,
                                                      msg.data,
                                                      msg.dlc,
                                                      &result->solution);

            if (complete == 1)
            {
                result->oem_frames_processed++;
            }

            result->bestpos_received =
                ((result->solution.data_valid_flags & 0x01U) != 0U) ? 1U : 0U;

            result->bestvel_received =
                ((result->solution.data_valid_flags & 0x02U) != 0U) ? 1U : 0U;

            result->time_received =
                ((result->solution.data_valid_flags & 0x04U) != 0U) ? 1U : 0U;

            if (oem719_is_solution_valid(&result->solution))
            {
                result->solution_valid = 1;
                return 0;
            }
        }
        else
        {
            /*
             * If no frames are arriving, retry configuration periodically.
             */
            TickType_t now = xTaskGetTickCount();

            if ((now - last_config_tx) >= config_retry_ticks)
            {
                if (oem719_set_message_rate(OEM719_TEST_MESSAGE_RATE_MS) != 0)
                {
                    result->can_tx_error = 1;
                }

                if (oem719_set_constellations(OEM719_TEST_CONSTELLATION_MASK) != 0)
                {
                    result->can_tx_error = 1;
                }

                last_config_tx = now;
            }
        }
    }

    result->rx_timeout = 1;

    return result->solution_valid ? 0 : -1;
}

/* ------------------------------------------------------------------------ */
/* Offline parser self-test helpers                                         */
/* ------------------------------------------------------------------------ */

static void test_pack_u16_le(uint8_t *data, uint8_t offset, uint16_t val)
{
    data[offset + 0] = (uint8_t)(val >> 0);
    data[offset + 1] = (uint8_t)(val >> 8);
}

static void test_pack_u32_le(uint8_t *data, uint8_t offset, uint32_t val)
{
    data[offset + 0] = (uint8_t)(val >> 0);
    data[offset + 1] = (uint8_t)(val >> 8);
    data[offset + 2] = (uint8_t)(val >> 16);
    data[offset + 3] = (uint8_t)(val >> 24);
}

static void test_pack_float_le(uint8_t *data, uint8_t offset, float val)
{
    uint32_t u;
    memcpy(&u, &val, sizeof(u));
    test_pack_u32_le(data, offset, u);
}

static void test_pack_double_le(uint8_t *data, uint8_t offset, double val)
{
    uint64_t u;
    memcpy(&u, &val, sizeof(u));

    test_pack_u32_le(data, offset + 0, (uint32_t)(u & 0xFFFFFFFFU));
    test_pack_u32_le(data, offset + 4, (uint32_t)(u >> 32));
}

static int test_float_eq(float a, float b)
{
    float d = a - b;

    if (d < 0.0f)
    {
        d = -d;
    }

    return d <= 1e-5f;
}

static int test_double_eq(double a, double b)
{
    double d = a - b;

    if (d < 0.0)
    {
        d = -d;
    }

    return d <= 1e-9;
}

/*
 * Offline parser test.
 *
 * This does not use real CAN hardware. It feeds synthetic CAN frames into
 * oem719_process_can_message() and checks that the BESTPOS/BESTVEL/TIME
 * data is parsed correctly.
 *
 * Note:
 * Your current oem719_driver.c does not populate position.seconds or
 * velocity.seconds, even though those fields exist in the header structures.
 * This test therefore does not verify those fields.
 */
int oem719_driver_parse_selftest(void)
{
    oem719_gnss_solution_t sol;
    uint8_t d[8];
    int rc;

    const uint32_t TEST_WEEK = 2254U;

    const double TEST_LATITUDE  = 51.123456789;
    const double TEST_LONGITUDE = -114.987654321;
    const double TEST_HEIGHT    = 1050.25;

    const float TEST_LAT_STD = 0.010f;
    const float TEST_LON_STD = 0.020f;
    const float TEST_HGT_STD = 0.030f;
    const float TEST_HDOP    = 0.80f;

    const uint32_t TEST_NUM_SATS = 14U;
    const uint16_t TEST_RESERVED = 0x1234U;

    const float TEST_HORIZ_SPEED = 12.34f;
    const float TEST_VERT_SPEED  = -0.50f;
    const float TEST_TRACK       = 270.50f;
    const float TEST_SPEED_STD   = 0.10f;
    const float TEST_TRACK_STD   = 0.20f;
    const float TEST_VDOP        = 0.90f;

    const uint32_t TEST_SECONDS     = 123456U;
    const int32_t  TEST_UTC_OFFSET  = -18;
    const uint16_t TEST_UTC_YEAR    = 2026U;
    const uint8_t  TEST_UTC_MONTH   = 4U;
    const uint8_t  TEST_UTC_DAY     = 28U;
    const uint8_t  TEST_UTC_HOUR    = 18U;
    const uint8_t  TEST_UTC_MINUTE  = 32U;
    const uint8_t  TEST_UTC_SECOND  = 10U;
    const uint8_t  TEST_LEAP_SECONDS = 18U;

    oem719_reset_parser();

    memset(&sol, 0, sizeof(sol));

    /*
     * Basic API sanity checks.
     */
    if (oem719_is_solution_valid(NULL) != 0)
    {
        return -1;
    }

    if (oem719_get_fix_status_string(FIX_STATUS_SOLUTION_COMPUTED) == NULL)
    {
        return -1;
    }

    if (oem719_get_pos_type_string(POS_TYPE_FIXED) == NULL)
    {
        return -1;
    }

    /* -------------------------------------------------------------------- */
    /* BESTPOS                                                              */
    /* -------------------------------------------------------------------- */

    /* Frame 0: Week, Status, Type */
    memset(d, 0, sizeof(d));
    test_pack_u32_le(d, 0, TEST_WEEK);
    d[4] = (uint8_t)FIX_STATUS_SOLUTION_COMPUTED;
    d[5] = (uint8_t)POS_TYPE_FIXED;

    rc = oem719_process_can_message(OEM719_BESTPOS_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 1: Latitude */
    memset(d, 0, sizeof(d));
    test_pack_double_le(d, 0, TEST_LATITUDE);

    rc = oem719_process_can_message(OEM719_BESTPOS_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 2: Longitude */
    memset(d, 0, sizeof(d));
    test_pack_double_le(d, 0, TEST_LONGITUDE);

    rc = oem719_process_can_message(OEM719_BESTPOS_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 3: Height */
    memset(d, 0, sizeof(d));
    test_pack_double_le(d, 0, TEST_HEIGHT);

    rc = oem719_process_can_message(OEM719_BESTPOS_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 4: lat_std, lon_std */
    memset(d, 0, sizeof(d));
    test_pack_float_le(d, 0, TEST_LAT_STD);
    test_pack_float_le(d, 4, TEST_LON_STD);

    rc = oem719_process_can_message(OEM719_BESTPOS_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 5: hgt_std, num_satellites */
    memset(d, 0, sizeof(d));
    test_pack_float_le(d, 0, TEST_HGT_STD);
    test_pack_u32_le(d, 4, TEST_NUM_SATS);

    rc = oem719_process_can_message(OEM719_BESTPOS_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 6: reserved, hdop. This should complete BESTPOS. */
    memset(d, 0, sizeof(d));
    test_pack_u16_le(d, 0, TEST_RESERVED);
    test_pack_float_le(d, 2, TEST_HDOP);

    rc = oem719_process_can_message(OEM719_BESTPOS_MSG_ID, d, 8, &sol);
    if (rc != 1)
    {
        return -1;
    }

    if ((sol.data_valid_flags & 0x01U) == 0U)
    {
        return -1;
    }

    /* -------------------------------------------------------------------- */
    /* BESTVEL                                                              */
    /* -------------------------------------------------------------------- */

    /* Frame 0: Week, Status, Type */
    memset(d, 0, sizeof(d));
    test_pack_u32_le(d, 0, TEST_WEEK);
    d[4] = (uint8_t)FIX_STATUS_SOLUTION_COMPUTED;
    d[5] = (uint8_t)POS_TYPE_FIXED;

    rc = oem719_process_can_message(OEM719_BESTVEL_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 1: horizontal_speed, vertical_speed */
    memset(d, 0, sizeof(d));
    test_pack_float_le(d, 0, TEST_HORIZ_SPEED);
    test_pack_float_le(d, 4, TEST_VERT_SPEED);

    rc = oem719_process_can_message(OEM719_BESTVEL_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 2: track, speed_std */
    memset(d, 0, sizeof(d));
    test_pack_float_le(d, 0, TEST_TRACK);
    test_pack_float_le(d, 4, TEST_SPEED_STD);

    rc = oem719_process_can_message(OEM719_BESTVEL_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 3: track_std, vdop. This should complete BESTVEL. */
    memset(d, 0, sizeof(d));
    test_pack_float_le(d, 0, TEST_TRACK_STD);
    test_pack_float_le(d, 4, TEST_VDOP);

    rc = oem719_process_can_message(OEM719_BESTVEL_MSG_ID, d, 8, &sol);
    if (rc != 1)
    {
        return -1;
    }

    if ((sol.data_valid_flags & 0x02U) == 0U)
    {
        return -1;
    }

    /* -------------------------------------------------------------------- */
    /* TIME                                                                 */
    /* -------------------------------------------------------------------- */

    /* Frame 0: week, seconds */
    memset(d, 0, sizeof(d));
    test_pack_u32_le(d, 0, TEST_WEEK);
    test_pack_u32_le(d, 4, TEST_SECONDS);

    rc = oem719_process_can_message(OEM719_TIME_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 1: utc_offset, utc_year, utc_month, utc_day */
    memset(d, 0, sizeof(d));
    test_pack_u32_le(d, 0, (uint32_t)TEST_UTC_OFFSET);
    test_pack_u16_le(d, 4, TEST_UTC_YEAR);
    d[6] = TEST_UTC_MONTH;
    d[7] = TEST_UTC_DAY;

    rc = oem719_process_can_message(OEM719_TIME_MSG_ID, d, 8, &sol);
    if (rc != 0)
    {
        return -1;
    }

    /* Frame 2: hour, minute, second, leap seconds. This completes TIME. */
    memset(d, 0, sizeof(d));
    d[0] = TEST_UTC_HOUR;
    d[1] = TEST_UTC_MINUTE;
    d[2] = TEST_UTC_SECOND;
    d[3] = TEST_LEAP_SECONDS;

    rc = oem719_process_can_message(OEM719_TIME_MSG_ID, d, 8, &sol);
    if (rc != 1)
    {
        return -1;
    }

    if ((sol.data_valid_flags & 0x04U) == 0U)
    {
        return -1;
    }

    /* -------------------------------------------------------------------- */
    /* Validate parsed values                                               */
    /* -------------------------------------------------------------------- */

    if (!oem719_is_solution_valid(&sol))
    {
        return -1;
    }

    if (sol.position.week != TEST_WEEK)
    {
        return -1;
    }

    if (sol.position.status != FIX_STATUS_SOLUTION_COMPUTED)
    {
        return -1;
    }

    if (sol.position.pos_type != POS_TYPE_FIXED)
    {
        return -1;
    }

    if (!test_double_eq(sol.position.latitude, TEST_LATITUDE))
    {
        return -1;
    }

    if (!test_double_eq(sol.position.longitude, TEST_LONGITUDE))
    {
        return -1;
    }

    if (!test_double_eq(sol.position.height, TEST_HEIGHT))
    {
        return -1;
    }

    if (!test_float_eq(sol.position.lat_std, TEST_LAT_STD))
    {
        return -1;
    }

    if (!test_float_eq(sol.position.lon_std, TEST_LON_STD))
    {
        return -1;
    }

    if (!test_float_eq(sol.position.hgt_std, TEST_HGT_STD))
    {
        return -1;
    }

    if (sol.position.num_satellites != TEST_NUM_SATS)
    {
        return -1;
    }

    if (sol.position.reserved != TEST_RESERVED)
    {
        return -1;
    }

    if (!test_float_eq(sol.position.hdop, TEST_HDOP))
    {
        return -1;
    }

    if (sol.velocity.week != TEST_WEEK)
    {
        return -1;
    }

    if (sol.velocity.status != FIX_STATUS_SOLUTION_COMPUTED)
    {
        return -1;
    }

    if (sol.velocity.pos_type != POS_TYPE_FIXED)
    {
        return -1;
    }

    if (!test_float_eq(sol.velocity.horizontal_speed, TEST_HORIZ_SPEED))
    {
        return -1;
    }

    if (!test_float_eq(sol.velocity.vertical_speed, TEST_VERT_SPEED))
    {
        return -1;
    }

    if (!test_float_eq(sol.velocity.track, TEST_TRACK))
    {
        return -1;
    }

    if (!test_float_eq(sol.velocity.speed_std, TEST_SPEED_STD))
    {
        return -1;
    }

    if (!test_float_eq(sol.velocity.track_std, TEST_TRACK_STD))
    {
        return -1;
    }

    if (!test_float_eq(sol.velocity.vdop, TEST_VDOP))
    {
        return -1;
    }

    if (sol.time.week != TEST_WEEK)
    {
        return -1;
    }

    if (sol.time.seconds != TEST_SECONDS)
    {
        return -1;
    }

    if (sol.time.utc_offset != TEST_UTC_OFFSET)
    {
        return -1;
    }

    if (sol.time.utc_year != TEST_UTC_YEAR)
    {
        return -1;
    }

    if (sol.time.utc_month != TEST_UTC_MONTH)
    {
        return -1;
    }

    if (sol.time.utc_day != TEST_UTC_DAY)
    {
        return -1;
    }

    if (sol.time.utc_hour != TEST_UTC_HOUR)
    {
        return -1;
    }

    if (sol.time.utc_minute != TEST_UTC_MINUTE)
    {
        return -1;
    }

    if (sol.time.utc_second != TEST_UTC_SECOND)
    {
        return -1;
    }

    if (sol.time.leap_seconds != TEST_LEAP_SECONDS)
    {
        return -1;
    }

    return 0;
}

void oem719_print_test_result(const oem719_test_result_t *result)
{
    if (result == NULL)
    {
        return;
    }

    printf("OEM719 CAN test results:\r\n");

    printf("  CAN init:                 %s\r\n",
           result->can_init_ok ? "PASS" : "FAIL");

    printf("  OEM719 driver init:       %s\r\n",
           result->driver_init_ok ? "PASS" : "FAIL");

    printf("  TX config message:        %s\r\n",
           result->tx_config_ok ? "PASS" : "FAIL");

    printf("  TX constellation config:  %s\r\n",
           result->tx_constellation_ok ? "PASS" : "FAIL");

    printf("  CAN TX error:             %s\r\n",
           result->can_tx_error ? "YES" : "NO");

    printf("  ACK received:             %s\r\n",
           result->ack_received ? "YES" : "NO");

    printf("  BESTPOS received:         %s\r\n",
           result->bestpos_received ? "YES" : "NO");

    printf("  BESTVEL received:         %s\r\n",
           result->bestvel_received ? "YES" : "NO");

    printf("  TIME received:            %s\r\n",
           result->time_received ? "YES" : "NO");

    printf("  Solution valid:           %s\r\n",
           result->solution_valid ? "YES" : "NO");

    printf("  RX timeout:               %s\r\n",
           result->rx_timeout ? "YES" : "NO");

    printf("  Total RX frames:          %lu\r\n",
           (unsigned long)result->rx_frames);

    printf("  OEM719 complete messages: %lu\r\n",
           (unsigned long)result->oem_frames_processed);

    printf("  Non-OEM719 frames:        %lu\r\n",
           (unsigned long)result->non_oem_frames);

    if (result->bestpos_received)
    {
        printf("  BESTPOS:\r\n");

        printf("    Week:        %lu\r\n",
               (unsigned long)result->solution.position.week);

        printf("    Latitude:    %.9f\r\n",
               result->solution.position.latitude);

        printf("    Longitude:   %.9f\r\n",
               result->solution.position.longitude);

        printf("    Height:      %.3f m\r\n",
               result->solution.position.height);

        printf("    Satellites:  %lu\r\n",
               (unsigned long)result->solution.position.num_satellites);

        printf("    HDOP:        %.2f\r\n",
               result->solution.position.hdop);

        printf("    Fix status:  %s\r\n",
               oem719_get_fix_status_string(result->solution.position.status));

        printf("    Pos type:    %s\r\n",
               oem719_get_pos_type_string(result->solution.position.pos_type));
    }

    if (result->bestvel_received)
    {
        printf("  BESTVEL:\r\n");

        printf("    Horizontal speed: %.3f m/s\r\n",
               result->solution.velocity.horizontal_speed);

        printf("    Vertical speed:   %.3f m/s\r\n",
               result->solution.velocity.vertical_speed);

        printf("    Track:            %.3f deg\r\n",
               result->solution.velocity.track);
    }

    if (result->time_received)
    {
        printf("  TIME:\r\n");

        printf("    UTC: %lu-%02lu-%02lu %02lu:%02lu:%02lu\r\n",
               (unsigned long)result->solution.time.utc_year,
               (unsigned long)result->solution.time.utc_month,
               (unsigned long)result->solution.time.utc_day,
               (unsigned long)result->solution.time.utc_hour,
               (unsigned long)result->solution.time.utc_minute,
               (unsigned long)result->solution.time.utc_second);

        printf("    GPS week:    %lu\r\n",
               (unsigned long)result->solution.time.week);

        printf("    GPS seconds: %lu\r\n",
               (unsigned long)result->solution.time.seconds);
    }
}
