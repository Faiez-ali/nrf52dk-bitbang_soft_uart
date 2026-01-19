#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include "soft_uart.h"
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
/* ================= RX BUFFER ================= */
#define RX_BUF_SIZE 64

static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_idx;

/* ================= RX CALLBACK =============== */
static void soft_uart_rx_cb(uint8_t byte)
{
    if (rx_idx < (RX_BUF_SIZE - 1)) {
        rx_buf[rx_idx++] = byte;
    }

    /* Print on newline or buffer full */
    if (byte == '\n' || rx_idx >= (RX_BUF_SIZE - 1)) {
        rx_buf[rx_idx] = '\0';
        LOG_INF("RX: %s", rx_buf);
        rx_idx = 0;
    }
}

/* ================= MAIN ====================== */
int main(void)
{
    int ret;
    uint32_t counter = 0;
    char tx_buf[64];

    LOG_INF("\nSoft UART Loopback Test Started\n");

    ret = soft_uart_init(soft_uart_rx_cb);
    if (ret) {
        LOG_INF("Soft UART init failed: %d\n", ret);
        return 0;
    }

    LOG_INF("Soft UART initialized OK\n");

    while (1) {
        snprintf(tx_buf, sizeof(tx_buf), "Hello %lu\r\n", counter++);
        LOG_INF("TX: %s", tx_buf);
        soft_uart_write_buf((uint8_t *)tx_buf, strlen(tx_buf));
        k_sleep(K_MSEC(1000));
    }
}
