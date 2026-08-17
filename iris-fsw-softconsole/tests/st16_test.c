//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File: st16_test.c
// Description:
//  Basic RS485 / NSP / ST-16 communication test.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <FreeRTOS-Kernel/include/FreeRTOS.h>
#include <FreeRTOS-Kernel/include/queue.h>
#include <FreeRTOS-Kernel/include/semphr.h>
#include <FreeRTOS-Kernel/include/task.h>

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "drivers/ST_16_driver.h"
#include "tests/st16_test.h"
#include "drivers/protocol/uart.h"

/*------------------------------------------------------------------------------------------------------------------------------
 * Configuration
 *------------------------------------------------------------------------------------------------------------------------------*/

/*
 * NSP address of this flight computer / test host
 */
#define ST16_TEST_MY_ADDR                   0x01

/*
 * Reply queue depth
 *
 * nsp_message_t is fairly large because it contains a 512-byte payload.
 * If heap is tight, reduce this. If using COMBINATION commands that generate
 * many replies, increase this
 */
#define ST16_TEST_REPLY_QUEUE_LEN           5

/*
 * Timeout for each command reply in milliseconds
 */
#define ST16_TEST_TIMEOUT_MS                500

/*
 * Number of retries per command
 */
#define ST16_TEST_RETRIES                   3

/*
 * Example READ EDAC parameters
 * Adjust if query requires different addresses/lengths
 */
#define ST16_TEST_EDAC_ADDR                 0x0000
#define ST16_TEST_EDAC_LEN                  4

/*
 * Example READ RESULT parameters
 * Adjust if query requires different addresses/lengths
 */
#define ST16_TEST_RESULT_ADDR               0x0000
#define ST16_TEST_RESULT_LEN                16

/*
 * GO command code
 */
#define ST16_TEST_GO_CODE                   0x0B

/*
 * COMBINATION parameters
 *
 */
#define ST16_TEST_COMBO_GO_CODE             0x12
#define ST16_TEST_COMBO_BITMAP              0x0007FF

/*
 * RS485 direction control
 *
 * If RS485 transceiver requires manual DE/RE control, redefine these
 * macros or replace them with function calls
 *
 * Ex:
 *
 *   extern void rs485_tx_enable(void);
 *   extern void rs485_tx_disable(void);
 *
 *   #define ST16_RS485_TX_ENABLE()   rs485_tx_enable()
 *   #define ST16_RS485_TX_DISABLE()  rs485_tx_disable()
 *
 * If transceiver is auto-direction (i think it is), leave these empty
 */
#ifndef ST16_RS485_TX_ENABLE
#define ST16_RS485_TX_ENABLE()              do { } while (0)
#endif

#ifndef ST16_RS485_TX_DISABLE
#define ST16_RS485_TX_DISABLE()             do { } while (0)
#endif

/*------------------------------------------------------------------------------------------------------------------------------
 * External UART hooks
 *------------------------------------------------------------------------------------------------------------------------------*/

extern mss_uart_instance_t g_mss_uart0;

/*
 * existing UART transmit function from uart.c
 *
 */
extern void prvUARTSend(mss_uart_instance_t *pxUART,
                        const uint8_t *pcBuffer,
                        size_t xBufferLength);

/*------------------------------------------------------------------------------------------------------------------------------
 * Private state
 *------------------------------------------------------------------------------------------------------------------------------*/

static st_driver_t s_st;
static QueueHandle_t s_reply_queue = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;
static volatile bool s_ready = false;

/*------------------------------------------------------------------------------------------------------------------------------
 * Private helpers
 *------------------------------------------------------------------------------------------------------------------------------*/

static void prvReplyCallback(const nsp_message_t *msg)
{
    if ((s_reply_queue == NULL) || (msg == NULL)) {
        return;
    }

    /*
     * message buffer is only valid during the callback, so copy it
     *
     * This callback is expected to be called from task context because
     * st_process_rx_bytes() should be called from the UART RX task
     *
     * If calling st_process_rx_bytes() from ISR context, use
     * xQueueSendFromISR() instead
     */
    nsp_message_t copy = *msg;
    (void)xQueueSend(s_reply_queue, &copy, 0);
}

static void prvRs485Send(const uint8_t *buf, size_t len)
{
    if ((buf == NULL) || (len == 0) || (s_tx_mutex == NULL)) {
        return;
    }

    if (xSemaphoreTake(s_tx_mutex, portMAX_DELAY) == pdTRUE) {
        ST16_RS485_TX_ENABLE();

        prvUARTSend(&g_mss_uart0, buf, len);

        /*
         * Small delay allows the final stop bit to leave the UART before
         * disabling the RS485 transmitter
         * Adjust if needed
         */
        vTaskDelay(pdMS_TO_TICKS(1));

        ST16_RS485_TX_DISABLE();

        xSemaphoreGive(s_tx_mutex);
    }
}

static void prvFlushReplyQueue(void)
{
    nsp_message_t msg;

    if (s_reply_queue == NULL) {
        return;
    }

    while (xQueueReceive(s_reply_queue, &msg, 0) == pdTRUE) {
        /* Discard stale replies. */
    }
}

static bool prvWaitReply(nsp_message_t *msg, TickType_t timeout)
{
    if ((s_reply_queue == NULL) || (msg == NULL)) {
        return false;
    }

    return (xQueueReceive(s_reply_queue, msg, timeout) == pdTRUE);
}

static void prvPrintReply(const nsp_message_t *msg)
{
    if (msg == NULL) {
        return;
    }

    printf("    src=0x%02X dest=0x%02X ctrl=0x%02X len=%u\n",
           msg->src,
           msg->dest,
           msg->control,
           msg->data_len);

    if (msg->data_len > 0) {
        uint16_t print_len = msg->data_len;

        if (print_len > 32) {
            print_len = 32;
        }

        printf("    data:  ");
        for (uint16_t i = 0; i < print_len; i++) {
            printf("%02X ", msg->data[i]);
        }
        printf("\n");

        printf("    ascii: ");
        for (uint16_t i = 0; i < print_len; i++) {
            uint8_t c = msg->data[i];
            putchar((c >= 0x20 && c < 0x7F) ? (int)c : '.');
        }

        if (msg->data_len > print_len) {
            printf(" ...");
        }

        printf("\n");
    }
}

static bool prvSendAndWait(const char *name,
                           uint8_t dest,
                           st_command_code_t cmd,
                           const uint8_t *data,
                           uint16_t data_len,
                           TickType_t timeout,
                           bool strict_src)
{
    nsp_message_t reply;

    if (name == NULL) {
        name = "UNKNOWN";
    }

    for (uint32_t attempt = 1; attempt <= ST16_TEST_RETRIES; attempt++) {
        prvFlushReplyQueue();

        printf("[ST16] %-24s attempt %u/%u\n",
               name,
               (unsigned)attempt,
               (unsigned)ST16_TEST_RETRIES);

        st_send_command(&s_st,
                        dest,
                        cmd,
                        data,
                        data_len,
                        true);

        if (prvWaitReply(&reply, timeout)) {
            prvPrintReply(&reply);

            if (strict_src && (reply.src != dest)) {
                printf("[ST16] FAIL: expected src 0x%02X, got src 0x%02X\n",
                       dest,
                       reply.src);
                continue;
            }

            if ((reply.control & NSP_CTRL_ACK) == 0) {
                printf("[ST16] WARNING: ACK bit not set in reply control byte 0x%02X\n",
                       reply.control);
            }

            if ((reply.control & NSP_CTRL_CMD_MASK) != ((uint8_t)cmd & NSP_CTRL_CMD_MASK)) {
                printf("[ST16] WARNING: reply command bits 0x%02X != sent command bits 0x%02X\n",
                       reply.control & NSP_CTRL_CMD_MASK,
                       ((uint8_t)cmd & NSP_CTRL_CMD_MASK));
            }

            return true;
        }

        printf("[ST16] %-24s timeout\n", name);
    }

    printf("[ST16] %-24s FAIL\n", name);
    return false;
}

/*------------------------------------------------------------------------------------------------------------------------------
 * Public API
 *------------------------------------------------------------------------------------------------------------------------------*/

bool st16_test_init(void)
{
    s_ready = false;

    if (s_reply_queue == NULL) {
        s_reply_queue = xQueueCreate(ST16_TEST_REPLY_QUEUE_LEN,
                                     sizeof(nsp_message_t));
    }

    if (s_tx_mutex == NULL) {
        s_tx_mutex = xSemaphoreCreateMutex();
    }

    if ((s_reply_queue == NULL) || (s_tx_mutex == NULL)) {
        printf("[ST16] Test init failed: FreeRTOS object allocation failure\n");
        return false;
    }

    st_init(&s_st,
            ST16_TEST_MY_ADDR,
            ST_SUPERVISOR_ADDR_DEFAULT,
            ST_FUNCTIONAL_ADDR_DEFAULT,
            prvRs485Send,
            prvReplyCallback);

    s_ready = true;

    printf("[ST16] Test driver initialized. my=0x%02X supervisor=0x%02X functional=0x%02X\n",
           s_st.my_nsp_addr,
           s_st.st_supervisor_addr,
           s_st.st_functional_addr);

    return true;
}

void st16_test_rx(const uint8_t *data, size_t len)
{
    if (!s_ready) {
        return;
    }

    if ((data == NULL) || (len == 0)) {
        return;
    }

    st_process_rx_bytes(&s_st, data, len);
}

bool st16_test_run(bool run_optional_commands)
{
    bool pass = true;
    TickType_t timeout = pdMS_TO_TICKS(ST16_TEST_TIMEOUT_MS);

    if (!s_ready) {
        printf("[ST16] Test not initialized\n");
        return false;
    }

    printf("\n");
    printf("========================================\n");
    printf(" ST-16 RS485 / NSP Basic Communication Test\n");
    printf("========================================\n");

    /*
     * PING supervisor.
     */
    pass &= prvSendAndWait("PING supervisor",
                           s_st.st_supervisor_addr,
                           ST_CMD_PING,
                           NULL,
                           0,
                           timeout,
                           true);

    /*
     * PING functional processor.
     */
    pass &= prvSendAndWait("PING functional",
                           s_st.st_functional_addr,
                           ST_CMD_PING,
                           NULL,
                           0,
                           timeout,
                           true);

    /*
     * READ TIME.
     */
    pass &= prvSendAndWait("READ TIME",
                           s_st.st_supervisor_addr,
                           ST_CMD_READ_TIME,
                           NULL,
                           0,
                           timeout,
                           true);

    /*
     * READ EDAC short.
     *
     * Payload:
     *   EDAC address low byte
     *   EDAC address high byte
     *   length
     */
    uint8_t edac_cmd[3];
    edac_cmd[0] = (uint8_t)(ST16_TEST_EDAC_ADDR & 0xFF);
    edac_cmd[1] = (uint8_t)((ST16_TEST_EDAC_ADDR >> 8) & 0xFF);
    edac_cmd[2] = ST16_TEST_EDAC_LEN;

    pass &= prvSendAndWait("READ EDAC short",
                           s_st.st_supervisor_addr,
                           ST_CMD_READ_EDAC,
                           edac_cmd,
                           sizeof(edac_cmd),
                           timeout,
                           true);

    /*
     * READ RESULT short.
     *
     * Payload:
     *   result address low byte
     *   result address high byte
     *   length
     */
    uint8_t result_cmd[3];
    result_cmd[0] = (uint8_t)(ST16_TEST_RESULT_ADDR & 0xFF);
    result_cmd[1] = (uint8_t)((ST16_TEST_RESULT_ADDR >> 8) & 0xFF);
    result_cmd[2] = ST16_TEST_RESULT_LEN;

    pass &= prvSendAndWait("READ RESULT short",
                           s_st.st_supervisor_addr,
                           ST_CMD_READ_RESULT,
                           result_cmd,
                           sizeof(result_cmd),
                           timeout,
                           true);

    if (run_optional_commands) {
        printf("\n");
        printf("[ST16] Running optional command tests.\n");
        printf("[ST16] Ensure GO / COMBINATION values are safe before flight use.\n");

        /*
         * GO.
         *
         * Replace ST16_TEST_GO_CODE with a valid GO code from your ICD.
         */
        uint8_t go_code = ST16_TEST_GO_CODE;

        pass &= prvSendAndWait("GO",
                               s_st.st_supervisor_addr,
                               ST_CMD_GO,
                               &go_code,
                               sizeof(go_code),
                               timeout,
                               true);

        /*
         * COMBINATION.
         *
         * st_combination() payload format:
         *   byte 0: GO code
         *   byte 1: bitmap bits 7:0
         *   byte 2: bitmap bits 15:8
         *   byte 3: bitmap bits 23:16
         *
         *
         */
        uint8_t combination_cmd[4];
        combination_cmd[0] = ST16_TEST_COMBO_GO_CODE;
        combination_cmd[1] = (uint8_t)(ST16_TEST_COMBO_BITMAP & 0xFF);
        combination_cmd[2] = (uint8_t)((ST16_TEST_COMBO_BITMAP >> 8) & 0xFF);
        combination_cmd[3] = (uint8_t)((ST16_TEST_COMBO_BITMAP >> 16) & 0xFF);

        pass &= prvSendAndWait("COMBINATION",
                               s_st.st_supervisor_addr,
                               ST_CMD_COMBINATION,
                               combination_cmd,
                               sizeof(combination_cmd),
                               timeout,
                               true);
    }

    printf("\n");
    printf("[ST16] Overall test result: %s\n", pass ? "PASS" : "FAIL");
    printf("========================================\n\n");

    return pass;
}

void vTaskST16Test(void *pvParameters)
{
    bool run_optional_commands = (pvParameters != NULL);

    if (!st16_test_init()) {
        printf("[ST16] Test initialization failed\n");
        vTaskDelete(NULL);
        return;
    }

    /*
     * Allow UART bridge / RX task to settle
     */
    vTaskDelay(pdMS_TO_TICKS(100));

    bool pass = st16_test_run(run_optional_commands);

    if (pass) {
        printf("[ST16] Test task complete: PASS\n");
    } else {
        printf("[ST16] Test task complete: FAIL\n");
    }

    vTaskDelete(NULL);
}
