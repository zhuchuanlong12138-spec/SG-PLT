/******************************************************************************
 * main.c (PAN3029 RAW RF TEST)
 *
 * 目的：先调通 PAN3029 的最小收发闭环，不走 DL/T 2807 协议栈。
 *
 * - UART1: dbg.c 负责串口打印与命令接收
 * - SysTick 1ms: delay.c 负责 tick
 * - RF: 使用 pan3029_rf.c/pan3029_rf.h（沿用 STM32 可通版本）
 * - 角色切换：COORD / NODE
 *
 * 文件结构：
 *   main.c            -> 时钟/串口/SysTick/调度/心跳
 *   APP_COORD_main.c  -> 协调器：常开 RX，收到后回 ACK
 *   APP_NODE_main.c   -> 节点：周期 TX，等待 ACK
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "ddl.h"
#include "clk.h"
#include "delay.h"
#include "dbg.h"

#include "pan3029_rf.h"
#include "pan3029_port.h"

/* Switch system clock to 32MHz external high-speed crystal (XTH).
 * Must be called BEFORE UART init (baud depends on PCLK).
 */
static void app_switch_clock_to_32mhz(void)
{
    Clk_Enable(ClkXTH, TRUE);
    while (FALSE == Clk_GetClkRdy(ClkXTH)) {
        ;
    }
    Clk_SwitchTo(ClkXTH);
    Clk_SetHClkDiv(ClkDiv1);
    Clk_SetPClkDiv(ClkDiv1);
}

/*==================== 身份切换（你只改这里） ====================*/
#define APP_ROLE_SELECT_COORD   (1u)
#define APP_ROLE_SELECT_NODE    (2u)

/* 选择身份 */
 //#define APP_ROLE_SELECT      (APP_ROLE_SELECT_NODE)
#define APP_ROLE_SELECT         (APP_ROLE_SELECT_COORD)

/*==================== 通用配置 ====================*/
#define DBG_UART_BAUD           (115200u)
#define HEARTBEAT_PERIOD_MS     (1000u)

/*==================== 全局（提供给 APP 调用） ====================*/
volatile uint32_t g_ms = 0u;

/*==================== SysTick 1ms ====================*/
void SysTick_Handler(void)
{
    g_ms++;
    Delay_IncTick();
}

/*==================== App 侧接口（由另外两个 .c 实现） ====================*/
void app_coord_init(void);
void app_coord_task(void);
void app_coord_on_cmd(uint8_t ch);

void app_node_init(void);
void app_node_task(void);
void app_node_on_cmd(uint8_t ch);

/*==================== main 自己的 help ====================*/
static void main_print_help(void)
{
    dbg_puts("\r\n================ MAIN HELP ================\r\n");
    dbg_puts("m : main help\r\n");
    dbg_puts("h/? : forward to role help\r\n");
    dbg_puts("==========================================\r\n");
}

/*==================== 心跳打印（含 RF 状态） ====================*/
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

    dbg_puts(" txf=");
    dbg_put_u32((uint32_t)rf_get_transmit_flag());

    dbg_puts(" rxf=");
    dbg_put_u32((uint32_t)rf_get_recv_flag());

    dbg_puts(" mode=");
    dbg_put_u32((uint32_t)rf_get_mode());

    dbg_puts(" freq=");
    dbg_put_u32((uint32_t)rf_read_freq());

    dbg_puts(" bw=");
    dbg_put_u32((uint32_t)rf_get_bw());

    dbg_puts(" sf=");
    dbg_put_u32((uint32_t)rf_get_sf());

    dbg_puts(" sync=0x");
    dbg_put_hex8(rf_get_syncword());

    dbg_puts("\r\n");
}

/*==================== main：统一调度入口 ====================*/
int32_t main(void)
{
    uint32_t last_hb = 0u;

    app_switch_clock_to_32mhz();
    dbg_uart_init(DBG_UART_BAUD);

    dbg_puts("\r\n\r\n==============================\r\n");
    dbg_puts("[BOOT] pan3029 raw-rf test start\r\n");
    dbg_puts("[BOOT] PCLK=");
    dbg_put_u32((uint32_t)Clk_GetPClkFreq());
    dbg_puts(" Hz\r\n");
    dbg_puts("[BOOT] UART1 Mode3 baud=");
    dbg_put_u32((uint32_t)DBG_UART_BAUD);
    dbg_puts("\r\n");

#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
    dbg_puts("[BOOT] ROLE=COORDINATOR\r\n");
#else
    dbg_puts("[BOOT] ROLE=NODE\r\n");
#endif
    dbg_puts("==============================\r\n");

    Delay_Init((uint32_t)Clk_GetHClkFreq());
		
		
		dbg_puts("[DBG] g_ms=");
		dbg_put_u32(g_ms);
		dbg_puts(" before rf_init\r\n");

		Delay_Ms(200);

		dbg_puts("[DBG] g_ms=");
		dbg_put_u32(g_ms);
		dbg_puts(" after 200ms\r\n");



    dbg_puts("[BOOT] Delay_Init(SysTick 1ms) OK\r\n");

    /* PAN3029 HW bring-up must happen BEFORE rf_init().
     * - SPI PinMux + SPI peripheral init
     * - CSN default high
     * - IRQ pin input + interrupt enable
     * - RST pin reset pulse
     */
    pan3029_port_init_once();
    pan3029_port_hw_reset();

    main_print_help();

#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
    app_coord_init();
#else
    app_node_init();
#endif

    last_hb = (uint32_t)g_ms;

    while (1)
    {
        /* 双保险：既支持 GPIO IRQ 回调里调用 rf_irq_process()，也支持轮询 */
        rf_irq_process();

#if (APP_ROLE_SELECT == APP_ROLE_SELECT_COORD)
        app_coord_task();
#else
        app_node_task();
#endif

        {
            uint8_t ch;
            if (dbg_getc(&ch))
            {
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
        }

        if ((uint32_t)(g_ms - last_hb) >= (uint32_t)HEARTBEAT_PERIOD_MS)
        {
            last_hb = (uint32_t)g_ms;
            print_heartbeat();
        }
    }
}
