#ifndef __DBG_H__
#define __DBG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
