#include "soft_uart.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <nrfx_clock.h> 
#include <zephyr/drivers/clock_control.h> 
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

LOG_MODULE_REGISTER(soft_uart, LOG_LEVEL_INF);

/* ================= CONFIG ================= */
#define SOFT_UART_BAUDRATE      2400
#define BIT_TIME_US             (1000000 / SOFT_UART_BAUDRATE)

/* ================= DEVICETREE ============= */
#define SOFT_UART_NODE DT_NODELABEL(soft_uart)

static const struct gpio_dt_spec tx_gpio =
    GPIO_DT_SPEC_GET(SOFT_UART_NODE, tx_gpios);
static const struct gpio_dt_spec rx_gpio =
    GPIO_DT_SPEC_GET(SOFT_UART_NODE, rx_gpios);

/* ================= RX STATE ================ */
static struct k_timer rx_timer;
static struct gpio_callback rx_cb;

static volatile uint8_t rx_byte;
static volatile uint8_t rx_bit_index;
static volatile bool rx_active;

static soft_uart_rx_cb_t user_rx_cb;

/* ================= RX TIMER ================ */
static void rx_sample_timer(struct k_timer *timer)
{
    int bit = gpio_pin_get_dt(&rx_gpio);

    if (rx_bit_index < 8) {
        rx_byte |= (bit << rx_bit_index);
        rx_bit_index++;
    } else {
        /* Stop bit sampled */
        rx_active = false;
        k_timer_stop(&rx_timer);

        if (user_rx_cb) {
            user_rx_cb(rx_byte);
        }
    }
}

/* ================= RX GPIO ISR ============= */
static void rx_start_bit_isr(const struct device *dev,
                             struct gpio_callback *cb,
                             uint32_t pins)
{
    if (rx_active) {
        return;
    }

    rx_active = true;
    rx_byte = 0;
    rx_bit_index = 0;

    /* Sample in middle of first data bit */
    k_timer_start(&rx_timer,
                  K_USEC(BIT_TIME_US + (BIT_TIME_US / 2)),
                  K_USEC(BIT_TIME_US));
}

/* ================= API ===================== */
int soft_uart_init(soft_uart_rx_cb_t cb)
{

        /* ----- enable HFCLK explicitly ----- */ 
    nrfx_clock_init(NULL); 
    nrfx_clock_hfclk_start();
    while (!nrfx_clock_hfclk_is_running())
    { 
        LOG_ERR("Waiting for HFCLK to start...");
        /* spin */
    }

    int ret;

    user_rx_cb = cb;

    if (!device_is_ready(tx_gpio.port) ||
        !device_is_ready(rx_gpio.port)) {
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&tx_gpio, GPIO_OUTPUT_HIGH);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&rx_gpio, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&rx_gpio,
                                          GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    gpio_init_callback(&rx_cb, rx_start_bit_isr, BIT(rx_gpio.pin));
    gpio_add_callback(rx_gpio.port, &rx_cb);

    k_timer_init(&rx_timer, rx_sample_timer, NULL);

    LOG_INF("Soft UART initialized, baud=%d", SOFT_UART_BAUDRATE);

    return 0;
}

void soft_uart_write_byte(uint8_t byte)
{
    /* Start bit */
    gpio_pin_set_dt(&tx_gpio, 0);
    k_busy_wait(BIT_TIME_US);

    /* Data bits (LSB first) */
    for (int i = 0; i < 8; i++) {
        gpio_pin_set_dt(&tx_gpio, (byte >> i) & 0x01);
        k_busy_wait(BIT_TIME_US);
    }

    /* Stop bit */
    gpio_pin_set_dt(&tx_gpio, 1);
    k_busy_wait(BIT_TIME_US);
}

void soft_uart_write_buf(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        soft_uart_write_byte(buf[i]);
    }
}
