#ifndef __DBG_UART_H__
#define __DBG_UART_H__

#include <stdint.h>

/* 初始化 UART1 (P35 TX, P36 RX) 用于调试打印 */
void dbg_uart_init(uint32_t baud);

/* 基础打印 */
void dbg_putc(char c);
void dbg_puts(const char *s);
void dbg_put_hex8(uint8_t v);
void dbg_put_hex16(uint16_t v);
void dbg_put_u32(uint32_t v);

#endif
