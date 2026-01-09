#include "ddl.h"
#include "clk.h"
#include "gpio.h"
#include "uart.h"
#include "bt.h"
#include "dbg_uart.h"

/* === 和你之前项目保持一致：UART1 + TIM1 做波特率 === */
#define DBG_UART_CH     (UARTCH1)
#define DBG_BT_TIM      (TIM1)

/* UART1 RX 回调（这里不接收也行，但保持结构一致） */
static void RxIntCallback(void)
{
    /* 清接收满标志，避免一直进中断（如果你启用 RX 中断） */
    Uart_ClrStatus(DBG_UART_CH, UartRxFull);
}

/* UART1 错误回调 */
static void ErrIntCallback(void)
{
    /* 清错误标志 */
    Uart_ClrStatus(DBG_UART_CH, UartRFRAMEError);
}

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

    if (baud == 0u)
    {
        baud = 115200u;
    }

    /* UART1 引脚复用：按你之前项目 */
    Gpio_SetFunc_UART1TX_P35();
    Gpio_SetFunc_UART1RX_P36();

    /* 外设时钟门控：UART1 + BT */
    Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);
    Clk_SetPeripheralGate(ClkPeripheralUart1, TRUE);

    /* 中断回调（如果你不想用中断，也可以不启用 RX 中断） */
    stcUartIrqCb.pfnRxIrqCb    = RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb    = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = ErrIntCallback;

    stcConfig.pstcIrqCb  = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;      /* 需要的话才开 NVIC */
    stcConfig.enRunMode  = UartMode3; /* 8N1, 可变波特率 */

    stcMulti.enMulti_mode   = UartNormal;
    stcConfig.pstcMultiMode = &stcMulti;

    /* 波特率参数：关键点！必须用 Uart_SetBaudRate + BT */
    stcBaud.bDbaud  = 0u;
    stcBaud.u32Baud = baud;
    stcBaud.u8Mode  = UartMode3;

    pclk  = Clk_GetPClkFreq();
    timer = Uart_SetBaudRate(DBG_UART_CH, pclk, &stcBaud);

    /* BT 定时器启动（TIM1） */
    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;

    Bt_Init(DBG_BT_TIM, &stcBtConfig);
    Bt_ARRSet(DBG_BT_TIM, timer);
    Bt_Cnt16Set(DBG_BT_TIM, timer);
    Bt_Run(DBG_BT_TIM);

    /* UART 初始化并开启接收功能（发送不依赖 RX 中断） */
    Uart_Init(DBG_UART_CH, &stcConfig);

    /* RX 中断可选：你想要打印其实不需要 RX，中断也可以不开
     * 但这里保持你之前项目一致，方便后续你要交互命令。
     */
    Uart_EnableIrq(DBG_UART_CH, UartRxIrq);
    Uart_ClrStatus(DBG_UART_CH, UartRxFull);
    Uart_EnableFunc(DBG_UART_CH, UartRx);

    dbg_puts("\r\n[DBG] UART1 init OK\r\n");
}

void dbg_putc(char c)
{
    /* 直接用轮询发送：Uart_SendData 内部已等待 TxEmpty */
    (void)Uart_SendData(DBG_UART_CH, (uint8_t)c);
}

void dbg_puts(const char *s)
{
    while (s && *s)
    {
        if (*s == '\n') dbg_putc('\r');
        dbg_putc(*s++);
    }
}

static char hex4(uint8_t v)
{
    v &= 0x0Fu;
    return (v < 10u) ? (char)('0' + v) : (char)('A' + (v - 10u));
}

void dbg_put_hex8(uint8_t v)
{
    dbg_putc(hex4(v >> 4));
    dbg_putc(hex4(v));
}

void dbg_put_hex16(uint16_t v)
{
    dbg_put_hex8((uint8_t)(v >> 8));
    dbg_put_hex8((uint8_t)(v & 0xFFu));
}

void dbg_put_u32(uint32_t v)
{
    char buf[11];
    int i = 0;

    if (v == 0u)
    {
        dbg_putc('0');
        return;
    }

    while (v > 0u && i < 10)
    {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (i--)
    {
        dbg_putc(buf[i]);
    }
}
