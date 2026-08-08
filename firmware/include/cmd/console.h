#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void console_init(void);
/** Feed RX bytes from UART IRQ / poll. */
void console_rx_byte(uint8_t b);
void console_poll(void);
void console_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
