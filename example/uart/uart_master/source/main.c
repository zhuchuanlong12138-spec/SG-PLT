/******************************************************************************
 * main.c
 * - UART1 Mode3：调试打印与命令（打印实现已移至 dbg.c）
 * - Delay_Init()：配置SysTick 1ms
 * - dl2807_mac_init() + dl2807_mac_task()：协议栈调度
 * - 通过宏切换身份：COORD / NODE
 *
 * 结构：
 *   main.c            -> 初始化 + 串口 + SysTick + 调度 + 统一打印
 *   APP_COORD_main.c  -> 协调器逻辑（REQ->RSP，打印）
 *   APP_NODE_main.c   -> 节点逻辑（发REQ，收RSP->ACK，打印）
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ddl.h"
#include "clk.h"
#include "gpio.h"
#include "uart.h"
#include "bt.h"
#include "delay.h"
#include "dbg.h"

#include "dl2807_mac.h"
#include "pan3029_rf.h"

/*==================== 身份切换（你只改这里） ====================*/
#define APP_ROLE_SELECT_COORD   (1u)
#define APP_ROLE_SELECT_NODE    (2u)

/* 选择身份 */
#define APP_ROLE_SELECT         (APP_ROLE_SELECT_NODE)
/* #define APP_ROLE_SELECT      (APP_ROLE_SELECT_COORD) */

/*==================== 通用配置 ====================*/
#define DBG_UART_CH             (UARTCH1)
#define DBG_UART_BAUD           (115200u)

#define HEARTBEAT_PERIOD_MS     (1000u)

/* 你原例程一致的IO（别删） */
#define T1_PORT                 (3)
#define T1_PIN                  (3)

/* 网络参数（两块板必须 PAN_ID 一致） */
#define APP_PAN_ID              (DL2807_MAC_DEFAULT_PAN_ID)

/* 协调器ID建议固定 0x0000，节点从 0x0001/0x0002... */
#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
#define APP_ROLE                (DL2807_ROLE_COORDINATOR)
#define APP_NODE_ID             (0x0000u)
#define APP_COORD_ID            (0x0000u)
#else
#define APP_ROLE                (DL2807_ROLE_NODE)
#define APP_NODE_ID             (0x0001u)   /* 节点ID：另一块板改成 0x0002 */
#define APP_COORD_ID            (0x0000u)
#endif

/*==================== 全局（提供给另外两个 .c 调用） ====================*/
dl2807_mac_ctx_t g_mac;
volatile uint32_t g_ms = 0u;

/*==================== UART RX（中断读SBUF） ====================*/
static volatile uint8_t g_rx_flag = 0u;
static volatile uint8_t g_rx_ch   = 0u;

static void RxIntCallback(void)
{
    g_rx_ch = (uint8_t)M0P_UART1->SBUF;
    g_rx_flag = 1u;
}

static void ErrIntCallback(void)
{
}

/*==================== SysTick 1ms ====================*/
void SysTick_Handler(void)
{
    g_ms++;
    Delay_IncTick();
    dl2807_mac_1ms_tick(&g_mac);
}

/*==================== App 侧接口（由另外两个 .c 实现） ====================*/
void app_coord_init(void);
void app_coord_task(void);
void app_coord_on_cmd(uint8_t ch);
void app_coord_on_frame(const dl2807_mac_frame_t *frame);

void app_node_init(void);
void app_node_task(void);
void app_node_on_cmd(uint8_t ch);
void app_node_on_frame(const dl2807_mac_frame_t *frame);

/*==================== main 自己的 help ====================*/
static void main_print_help(void)
{
    dbg_puts("\r\n================ MAIN HELP ================\r\n");
    dbg_puts("m : main help\r\n");
    dbg_puts("h/? : forward to role help\r\n");
    dbg_puts("==========================================\r\n");
}

/*==================== DL2807 收包回调：统一在 main 打印 + 转发APP ====================*/
void dl2807_mac_on_frame_indication(const dl2807_mac_ctx_t *ctx,
                                    const dl2807_mac_frame_t *frame)
{
    (void)ctx;

    dbg_puts("\r\n[DL2807][RX] t=");
    dbg_put_u32(g_ms);

    dbg_puts(" type=0x");
    dbg_put_hex8(frame->type);

    dbg_puts(" seq=");
    dbg_put_u32((uint32_t)frame->seq);

    dbg_puts(" pan=0x");
    dbg_put_hex16(frame->pan_id);

    dbg_puts(" src=0x");
    dbg_put_hex16(frame->src);

    dbg_puts(" dst=0x");
    dbg_put_hex16(frame->dst);

    dbg_puts(" len=");
    dbg_put_u32((uint32_t)frame->payload_len);

    dbg_puts(" crc=0x");
    dbg_put_hex16(frame->crc16);

    dbg_puts("\r\n");

    if (frame->payload_len > 0u)
    {
        dbg_puts("[DL2807][RX] payload: ");
        dbg_hexdump(frame->payload, frame->payload_len);
        dbg_puts("\r\n");
    }

#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
    app_coord_on_frame(frame);
#else
    app_node_on_frame(frame);
#endif
}

/*==================== UART1 初始化：官方例程方式 ====================*/
static void uart1_init_official(uint32_t baud)
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

    Gpio_InitIO(T1_PORT, T1_PIN, GpioDirIn);
    Gpio_InitIO(0, 3, GpioDirOut);
    Gpio_SetIO(0, 3, 1);

    Gpio_InitIOExt(3, 5, GpioDirOut, TRUE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(3, 6, GpioDirOut, TRUE, FALSE, FALSE, FALSE);

    Gpio_SetFunc_UART1TX_P35();
    Gpio_SetFunc_UART1RX_P36();

    Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);
    Clk_SetPeripheralGate(ClkPeripheralUart1, TRUE);

    stcUartIrqCb.pfnRxIrqCb    = RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb    = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = ErrIntCallback;

    stcConfig.pstcIrqCb  = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
    stcConfig.enRunMode  = UartMode3;

    stcMulti.enMulti_mode   = UartNormal;
    stcConfig.pstcMultiMode = &stcMulti;

    stcBaud.bDbaud  = 0u;
    stcBaud.u32Baud = baud;
    stcBaud.u8Mode  = UartMode3;

    pclk  = Clk_GetPClkFreq();
    timer = Uart_SetBaudRate(UARTCH1, pclk, &stcBaud);

    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;

    Bt_Init(TIM1, &stcBtConfig);
    Bt_ARRSet(TIM1, timer);
    Bt_Cnt16Set(TIM1, timer);
    Bt_Run(TIM1);

    Uart_Init(UARTCH1, &stcConfig);
    Uart_EnableIrq(UARTCH1, UartRxIrq);
    Uart_ClrStatus(UARTCH1, UartRxFull);
    Uart_EnableFunc(UARTCH1, UartRx);
}

/*==================== 心跳打印（含CTRL状态） ====================*/
static void print_heartbeat(void)
{
    dbg_puts("[HB] ms=");
    dbg_put_u32((uint32_t)g_ms);

    dbg_puts(" role=");
#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
    dbg_puts("COORD");
#else
    dbg_puts("NODE");
#endif

    dbg_puts(" state=");
    dbg_put_u32((uint32_t)g_mac.state);

    dbg_puts(" ch=");
    dbg_put_u32((uint32_t)g_mac.current_ch);

    dbg_puts(" synced=");
    dbg_put_u32((uint32_t)(g_mac.synced ? 1u : 0u));

    dbg_puts(" txp=");
    dbg_put_u32((uint32_t)(g_mac.tx_pending ? 1u : 0u));

    dbg_puts(" rxp=");
    dbg_put_u32((uint32_t)(g_mac.rx_pending ? 1u : 0u));

    dbg_puts(" txf=");
    dbg_put_u32((uint32_t)rf_get_transmit_flag());

    dbg_puts(" rxf=");
    dbg_put_u32((uint32_t)rf_get_recv_flag());

    dbg_puts(" ctrl_wait=");
    dbg_put_u32((uint32_t)(g_mac.ctrl_wait_rsp ? 1u : 0u));

    dbg_puts(" retry=");
    dbg_put_u32((uint32_t)g_mac.ctrl_req_retry);

    dbg_puts("\r\n");
}

/*==================== main：统一调度入口 ====================*/
int32_t main(void)
{
    uint32_t last_hb;
    uint32_t hclk;

    last_hb = 0u;
    hclk    = 0u;

    uart1_init_official(DBG_UART_BAUD);

    dbg_puts("\r\n\r\n==============================\r\n");
    dbg_puts("[BOOT] dl2807 split-main start\r\n");
    dbg_puts("[BOOT] PCLK=");
    dbg_put_u32((uint32_t)Clk_GetPClkFreq());
    dbg_puts(" Hz\r\n");

    dbg_puts("[BOOT] UART1 Mode3 baud=");
    dbg_put_u32((uint32_t)DBG_UART_BAUD);
    dbg_puts(" (2400 8E1)\r\n");

#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
    dbg_puts("[BOOT] ROLE=COORDINATOR\r\n");
#else
    dbg_puts("[BOOT] ROLE=NODE\r\n");
#endif
    dbg_puts("==============================\r\n");

    hclk = (uint32_t)Clk_GetHClkFreq();
    Delay_Init(hclk);
    dbg_puts("[BOOT] Delay_Init(SysTick 1ms) OK\r\n");

    dbg_puts("[BOOT] dl2807_mac_init...\r\n");
    dl2807_mac_init(&g_mac, APP_ROLE, APP_PAN_ID, APP_NODE_ID, APP_COORD_ID);
    dbg_puts("[BOOT] dl2807_mac_init OK\r\n");

    main_print_help();

#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
    app_coord_init();
#else
    app_node_init();
#endif

    last_hb = (uint32_t)g_ms;

    while (1)
    {
        dl2807_mac_task(&g_mac);

#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
        app_coord_task();
#else
        app_node_task();
#endif

        if (g_rx_flag)
        {
            uint8_t ch;
            ch = g_rx_ch;
            g_rx_flag = 0u;

            if (ch == (uint8_t)'m')
            {
                main_print_help();
            }
            else
            {
#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
                app_coord_on_cmd(ch);
#else
                app_node_on_cmd(ch);
#endif
            }
        }

        if ((uint32_t)(g_ms - last_hb) >= (uint32_t)HEARTBEAT_PERIOD_MS)
        {
            last_hb = (uint32_t)g_ms;
            print_heartbeat();
        }
    }
}
