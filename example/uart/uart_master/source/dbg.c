#include "dbg.h"

#include <stdint.h>

#include "ddl.h"
#include "clk.h"
#include "gpio.h"
#include "uart.h"
#include "bt.h"

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

/*==================== UART RX（中断读SBUF，缓存 1 字节） ====================*/
static volatile uint8_t s_rx_flag = 0u;
static volatile uint8_t s_rx_ch   = 0u;

static void RxIntCallback(void)
{
    /* 官方例程：RX 中断直接读 SBUF 清中断 */
    s_rx_ch = (uint8_t)M0P_UART1->SBUF;
    s_rx_flag = 1u;
}

static void ErrIntCallback(void)
{
    /* 暂不处理错误中断（如需可在这里加打印/计数） */
}

bool dbg_getc(uint8_t *ch)
{
    if (ch == 0)
    {
        return false;
    }

    if (s_rx_flag == 0u)
    {
        return false;
    }

    *ch = s_rx_ch;
    s_rx_flag = 0u;
    return true;
}

/*==================== UART1 初始化：从 main 移入 dbg ====================*/
void dbg_uart_init(uint32_t baud)
{
    uint16_t timer;
    uint32_t pclk;

    stc_uart_config_t         stcConfig;
    stc_uart_irq_cb_t         stcUartIrqCb;
    stc_uart_multimode_t      stcMulti;
    stc_uart_baud_config_t    stcBaud;
    stc_bt_config_t           stcBtConfig;

    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtConfig);

    timer = 0u;
    pclk  = 0u;

    /* 你原例程一致的IO（别删） */
    Gpio_InitIO(3, 3, GpioDirIn);
    Gpio_InitIO(0, 3, GpioDirOut);
    Gpio_SetIO(0, 3, 1);

    Gpio_InitIOExt(3, 5, GpioDirOut, TRUE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(3, 6, GpioDirOut, TRUE, FALSE, FALSE, FALSE);

    /* UART1: P35=TX, P36=RX */
    Gpio_SetFunc_UART1TX_P35();
    Gpio_SetFunc_UART1RX_P36();

    /* 打开外设时钟门控 */
    Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);
    Clk_SetPeripheralGate(ClkPeripheralUart1, TRUE);

    /* 绑定中断回调 */
    stcUartIrqCb.pfnRxIrqCb    = RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb    = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = ErrIntCallback;

    stcConfig.pstcIrqCb  = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
    stcConfig.enRunMode  = UartMode3;

    stcMulti.enMulti_mode   = UartNormal;
    stcConfig.pstcMultiMode = &stcMulti;

    /* 波特率配置 */
    stcBaud.bDbaud  = 0u;
    stcBaud.u32Baud = baud;
    stcBaud.u8Mode  = UartMode3;

    pclk  = Clk_GetPClkFreq();
    timer = Uart_SetBaudRate(UARTCH1, pclk, &stcBaud);

    /* BT 作为 UART1 波特率定时器 */
    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;

    Bt_Init(TIM1, &stcBtConfig);
    Bt_ARRSet(TIM1, timer);
    Bt_Cnt16Set(TIM1, timer);
    Bt_Run(TIM1);

    /* UART 初始化并开启 RX 中断 */
    Uart_Init(UARTCH1, &stcConfig);
    Uart_EnableIrq(UARTCH1, UartRxIrq);
    Uart_ClrStatus(UARTCH1, UartRxFull);
    Uart_EnableFunc(UARTCH1, UartRx);
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
void dbg_hex8(const char *tag, uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    char buf[64];
    int n = 0;

    while (tag && *tag && n < (int)(sizeof(buf) - 1)) buf[n++] = *tag++;

    buf[n++] = ':';
    buf[n++] = '0';
    buf[n++] = 'x';
    buf[n++] = hex[(v >> 4) & 0x0F];
    buf[n++] = hex[(v >> 0) & 0x0F];
    buf[n++] = '\r';
    buf[n++] = '\n';
    buf[n]   = '\0';

    dbg_puts(buf);
}
