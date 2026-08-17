//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File: st16_test.h
// Description:
//  Basic RS485 / NSP / ST-16 communication test.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#ifndef DRIVERS_ST16_TEST_H
#define DRIVERS_ST16_TEST_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * Initialize the ST-16 test driver.
 *
 * This creates the reply queue and tx mutex, and initializes the ST-16
 * driver using the UART RS485 send wrapper.
 *
 * Returns true on success.
 */
bool st16_test_init(void);

/*
 * Feed received UART bytes into the ST-16 SLIP decoder.
 *
 * Call this from your UART RX task / bridge task whenever bytes are
 * received from the ST-16 UART / RS485 port.
 */
void st16_test_rx(const uint8_t *data, size_t len);

/*
 * Run the basic ST-16 communication test.
 *
 * run_optional_commands:
 *  false: run safe basic commands only.
 *  true:  also run GO / COMBINATION using the placeholder values in st16_test.c.
 *
 * Returns true if all executed tests received valid replies.
 */
bool st16_test_run(bool run_optional_commands);

/*
 * Optional FreeRTOS task wrapper.
 *
 * Pass NULL as pvParameters for safe tests only.
 * Pass non-NULL to also run optional GO / COMBINATION commands.
 */
void vTaskST16Test(void *pvParameters);

#endif /* DRIVERS_ST16_TEST_H */
