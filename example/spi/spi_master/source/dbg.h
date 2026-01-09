#ifndef __DBG_H__
#define __DBG_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化调试串口（UART1 Mode3，偶校验），并开启RX中断收字节
 * @param baud 波特率（例如 115200 / 128000）
 */
void dbg_init(uint32_t baud);

/**
 * @brief 非阻塞获取1字节（由UART RX中断写入内部缓存）
 * @param ch 输出字节指针
 * @return true=拿到字节，false=当前无数据
 */
bool dbg_getch_nonblock(uint8_t *ch);

/* ===== 基础输出 ===== */
void dbg_send_byte(uint8_t b);
void dbg_puts(const char *s);

/* ===== 数字输出 ===== */
void dbg_put_hex8(uint8_t v);
void dbg_put_hex16(uint16_t v);
void dbg_put_u32(uint32_t v);

/* ===== dump ===== */
void dbg_hexdump(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DBG_H__ */
