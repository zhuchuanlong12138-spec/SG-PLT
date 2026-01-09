/******************************************************************************
 * main.c  (Clock->32MHz + PAN3029 SPI ONLY test)
 * - 先切换系统时钟到 32MHz（你已解决）
 * - 再初始化 Delay / dbg_uart（避免波特率被时钟切换影响）
 * - 再跑 PAN3029 SPI 读写自检 + CLI
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "ddl.h"
#include "clk.h"
#include "delay.h"
#include "dbg.h"

#include "pan3029_spi_test.h"

/* 你的串口波特率 */
#define DBG_UART_BAUD (115200u)

/*==================== 1ms tick（如果你需要 delay.c 的 SysTick 支持） ====================*/
volatile uint32_t g_ms = 0u;
void SysTick_Handler(void)
{
    g_ms++;
    Delay_IncTick();
}

/* =========================
 * 把你已经调通的“切到32M”的代码放这里
 * ========================= */
static void clock_switch_to_32m(void)
{
    /* ✅ 下面是“通用写法”，但你必须以你现在已验证的那段为准
     * 目标：最终 Clk_GetHClkFreq()/Clk_GetPClkFreq() 显示 32000000
     *
     * 如果你现在切 32M 是：
     * - 外部高速晶振 XTH = 32MHz：就用 Clk_SwitchTo(ClkXTH)
     * - 或其它方案（PLL/分频）：就用你已解决的方案
     */

    /* 典型：切到外部高速 XTH（32MHz） */
    (void)Clk_SwitchTo(ClkXTH);

    /* 建议：分频都设为 1，保证 HCLK/PCLK = 32MHz */
    Clk_SetHClkDiv(ClkDiv1);
    Clk_SetPClkDiv(ClkDiv1);

    SystemCoreClockUpdate();
}

int main(void)
{
    uint32_t hclk, pclk;

    /* 1) 先切时钟到 32M（必须在 dbg_init 之前！） */
    clock_switch_to_32m();

    /* 2) 更新频率 & 初始化 Delay（按当前 HCLK） */
    hclk = Clk_GetHClkFreq();
    pclk = Clk_GetPClkFreq();
    Delay_Init(hclk);

    /* 3) 再初始化调试串口（避免波特率被切时钟影响） */
    dbg_init(DBG_UART_BAUD);

    dbg_puts("\r\n==============================\r\n");
    dbg_puts("[BOOT] Clock switched OK\r\n");
    dbg_puts("[BOOT] HCLK="); dbg_put_u32(hclk); dbg_puts(" Hz\r\n");
    dbg_puts("[BOOT] PCLK="); dbg_put_u32(pclk); dbg_puts(" Hz\r\n");
    dbg_puts("[BOOT] UART baud="); dbg_put_u32(DBG_UART_BAUD); dbg_puts("\r\n");
    dbg_puts("==============================\r\n");

    /* 4) SPI + PAN3029 最小读写自检（page 写回读） */
    pan3029_spi_test_init();
    (void)pan3029_spi_sanity_test(true);

    dbg_puts("Type 'h' for SPI CLI\r\n");

    while (1)
    {
        /* 串口命令：h/i/t/x/rAA/wAAVV/dAANN/pN ... */
        pan3029_spi_cli_poll();
    }
}
