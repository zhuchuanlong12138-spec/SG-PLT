#include "dbg.h"

#include <stdint.h>
#include <stdbool.h>

#include "ddl.h"
#include "clk.h"
#include "gpio.h"
#include "uart.h"
#include "bt.h"

/*==================== UART1 RX：中断缓存 ====================*/
static volatile uint8_t s_rx_flag = 0u;
static volatile uint8_t s_rx_ch   = 0u;

static void RxIntCallback(void)
{
    /* 读SBUF清中断源（按你原main.c写法） */
    s_rx_ch   = (uint8_t)M0P_UART1->SBUF;
    s_rx_flag = 1u;
}

static void ErrIntCallback(void)
{
    /* 可按需增加错误统计/清标志 */
}

/*==================== UART1 初始化（官方例程方式：Mode3 + BT定时器） ====================*/
static void dbg_uart1_init_mode3(uint32_t baud)
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

    /* 你原工程里保留的IO初始化（别删） */
    Gpio_InitIO(3, 3, GpioDirIn);
    Gpio_InitIO(0, 3, GpioDirOut);
    Gpio_SetIO(0, 3, 1);

    Gpio_InitIOExt(3, 5, GpioDirOut, TRUE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(3, 6, GpioDirOut, TRUE, FALSE, FALSE, FALSE);

    /* UART1 Pinmux：P35 TX / P36 RX */
    Gpio_SetFunc_UART1TX_P35();
    Gpio_SetFunc_UART1RX_P36();

    /* 外设门控 */
    Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);
    Clk_SetPeripheralGate(ClkPeripheralUart1, TRUE);

    /* 中断回调 */
    stcUartIrqCb.pfnRxIrqCb    = RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb    = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = ErrIntCallback;

    stcConfig.pstcIrqCb  = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
    stcConfig.enRunMode  = UartMode3;

    stcMulti.enMulti_mode   = UartNormal;
    stcConfig.pstcMultiMode = &stcMulti;

    /* 波特率计算基于PCLK —— 所以必须在“切外部晶振后”再调用本函数 */
    stcBaud.bDbaud  = 0u;
    stcBaud.u32Baud = baud;
    stcBaud.u8Mode  = UartMode3;

    pclk  = Clk_GetPClkFreq();
    timer = Uart_SetBaudRate(UARTCH1, pclk, &stcBaud);

    /* BT TIM1 作为波特率发生器 */
    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;

    Bt_Init(TIM1, &stcBtConfig);
    Bt_ARRSet(TIM1, timer);
    Bt_Cnt16Set(TIM1, timer);
    Bt_Run(TIM1);

    /* UART init + 开RX中断 + 开接收 */
    Uart_Init(UARTCH1, &stcConfig);
    Uart_EnableIrq(UARTCH1, UartRxIrq);
    Uart_ClrStatus(UARTCH1, UartRxFull);
    Uart_EnableFunc(UARTCH1, UartRx);
}

/*==================== 对外API ====================*/
void dbg_init(uint32_t baud)
{
    /* 清缓存状态 */
    s_rx_flag = 0u;
    s_rx_ch   = 0u;

    dbg_uart1_init_mode3(baud);
}

bool dbg_getch_nonblock(uint8_t *ch)
{
    if (ch == 0)
    {
        return false;
    }

    if (s_rx_flag)
    {
        *ch = s_rx_ch;
        s_rx_flag = 0u;
        return true;
    }

    return false;
}

/*==================== 打印实现（沿用你当前dbg.c） ====================*/
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
