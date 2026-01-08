#include "dbg.h"
#include "uart.h"
#include <stdint.h>

static void dbg_put_hex4(uint8_t v)
{
    v = (uint8_t)(v & 0x0Fu);
    if (v < 10u)
    {
        dbg_send_byte((uint8_t)('0' + v));
    }
    else
    {
        dbg_send_byte((uint8_t)('A' + (v - 10u)));
    }
}

void dbg_send_byte(uint8_t b)
{
    /* UART1 Mode3：偶校验 TB8 */
    Uart_SetTb8(UARTCH1, Even, b);
    Uart_SendData(UARTCH1, b);
}

void dbg_puts(const char *s)
{
    if (s == 0)
    {
        return;
    }

    while (*s != '\0')
    {
        dbg_send_byte((uint8_t)(*s));
        s++;
    }
}

void dbg_put_hex8(uint8_t v)
{
    dbg_put_hex4((uint8_t)(v >> 4));
    dbg_put_hex4(v);
}

void dbg_put_hex16(uint16_t v)
{
    dbg_put_hex8((uint8_t)(v >> 8));
    dbg_put_hex8((uint8_t)(v & 0xFFu));
}

void dbg_put_u32(uint32_t v)
{
    char buf[11];
    uint8_t i;

    i = 0u;

    if (v == 0u)
    {
        dbg_send_byte((uint8_t)'0');
        return;
    }

    while ((v > 0u) && (i < (uint8_t)sizeof(buf)))
    {
        buf[i] = (char)('0' + (v % 10u));
        i++;
        v /= 10u;
    }

    while (i > 0u)
    {
        i--;
        dbg_send_byte((uint8_t)buf[i]);
    }
}

void dbg_hexdump(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0)
    {
        return;
    }

    for (i = 0u; i < len; i++)
    {
        dbg_put_hex8(data[i]);
        if (i != (uint16_t)(len - 1u))
        {
            dbg_send_byte((uint8_t)' ');
        }
    }
}
