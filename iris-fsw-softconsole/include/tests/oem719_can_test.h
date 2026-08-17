#ifndef OEM719_CAN_TEST_H
#define OEM719_CAN_TEST_H

#include <stdint.h>

#include "drivers/protocol/can.h"
#include "drivers/oem719_driver.h"

/**
 * Result structure for the OEM719 CAN communication test.
 */
typedef struct
{
    uint8_t can_init_ok;
    uint8_t driver_init_ok;

    uint8_t tx_config_ok;
    uint8_t tx_constellation_ok;
    uint8_t ack_received;

    uint8_t bestpos_received;
    uint8_t bestvel_received;
    uint8_t time_received;

    uint8_t solution_valid;
    uint8_t can_tx_error;
    uint8_t rx_timeout;

    uint32_t rx_frames;
    uint32_t oem_frames_processed;
    uint32_t non_oem_frames;

    oem719_gnss_solution_t solution;
} oem719_test_result_t;

/**
 * CAN send glue function used by oem719_driver.c.
 *
 * Your oem719_driver.c calls:
 *
 *     can_send(OEM719_CONFIG_MSG_ID, data, 4);
 *
 * The test file provides a weak implementation. If you already implement
 * can_send() in can.c, your strong implementation will override it.
 *
 * Returns 0 on success, negative on failure.
 */
int can_send(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * Main OEM719 CAN communication and basic GNSS functionality test.
 *
 * If rx_queue is NULL:
 *   - The test uses can_rx_queue.
 *   - If can_rx_queue is NULL, it calls init_CAN(baud, NULL).
 *
 * If rx_queue is non-NULL:
 *   - The test uses the provided queue and does not call init_CAN().
 *
 * Returns 0 if a complete GNSS solution became valid.
 */
int oem719_can_comm_test(CANBaudRate baud,
                         QueueHandle_t rx_queue,
                         uint32_t timeout_ms,
                         oem719_test_result_t *result);

/**
 * Offline parser self-test.
 *
 * Feeds synthetic BESTPOS/BESTVEL/TIME frames into
 * oem719_process_can_message() and verifies that the driver parses them.
 *
 * Returns 0 on success.
 */
int oem719_driver_parse_selftest(void);

/**
 * Optional debug print helper.
 */
void oem719_print_test_result(const oem719_test_result_t *result);

/**
 * Optional parser reset.
 *
 * Your current oem719_driver.c does not expose a parser reset. The test file
 * provides a weak stub so the code can compile. For robust repeated testing,
 * implement a real version in oem719_driver.c.
 */
void oem719_reset_parser(void);

#endif /* OEM719_CAN_TEST_H */
