#ifndef __DBG_H__
#define __DBG_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * dbg 模块职责：
 * 1) 统一负责 UART1(Mode3) 的初始化/收发；
 * 2) 提供最基础的调试打印接口（dbg_puts/hex/dec/hexdump）；
 * 3) 提供一个“非阻塞取命令字符”的接口给 main/app 使用。
 */

/* 初始化 UART1（Mode3，偶校验），并开启 RX 中断缓存 1 字节 */
void dbg_uart_init(uint32_t baud);

/* 非阻塞读取 1 字节：有数据返回 true，并把字符写入 *ch */
bool dbg_getc(uint8_t *ch);

/* 发送/打印 */
void dbg_send_byte(uint8_t b);
void dbg_puts(const char *s);

void dbg_put_hex8(uint8_t v);
void dbg_put_hex16(uint16_t v);
void dbg_put_u32(uint32_t v);

void dbg_hexdump(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DBG_H__ */
