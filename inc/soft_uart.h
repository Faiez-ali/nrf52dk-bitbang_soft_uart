#ifndef SOFT_UART_H
#define SOFT_UART_H

#include <stdint.h>
#include <zephyr/logging/log.h>


typedef void (*soft_uart_rx_cb_t)(uint8_t byte);

int soft_uart_init(soft_uart_rx_cb_t cb);
void soft_uart_write_byte(uint8_t byte);
void soft_uart_write_buf(const uint8_t *buf, uint16_t len);


#endif /* SOFT_UART_H */