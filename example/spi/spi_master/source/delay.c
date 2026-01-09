#include "delay.h"

/* ======================= Cortex-M0+ SysTick 寄存器(标准地址) ======================= */
#define SYST_CSR   (*((volatile uint32_t *)0xE000E010u)) /* Control and Status */
#define SYST_RVR   (*((volatile uint32_t *)0xE000E014u)) /* Reload Value */
#define SYST_CVR   (*((volatile uint32_t *)0xE000E018u)) /* Current Value */
#define SYST_CALIB (*((volatile uint32_t *)0xE000E01Cu)) /* Calibration */

#define SYST_CSR_ENABLE_Pos     (0u)
#define SYST_CSR_TICKINT_Pos    (1u)
#define SYST_CSR_CLKSOURCE_Pos  (2u)

#define SYST_CSR_ENABLE         (1u << SYST_CSR_ENABLE_Pos)
#define SYST_CSR_TICKINT        (1u << SYST_CSR_TICKINT_Pos)
#define SYST_CSR_CLKSOURCE      (1u << SYST_CSR_CLKSOURCE_Pos)

/* ======================= 模块内部变量 ======================= */
static volatile uint32_t g_ms_tick = 0u;
static uint32_t g_sysclk_hz = 0u;

/* 兼容 ARMCC5：NOP */
static void delay_nop(void)
{
    __asm { NOP }
}

/* 配置 SysTick：每 1ms 产生一次中断 */
static void delay_systick_config_1ms(uint32_t sysclk_hz)
{
    uint32_t reload;

    /* 1ms reload = sysclk/1000 - 1 */
    reload = (sysclk_hz / 1000u);
    if (reload == 0u)
    {
        reload = 1u;
    }
    reload = reload - 1u;

    /* SysTick 是 24-bit */
    if (reload > 0x00FFFFFFu)
    {
        reload = 0x00FFFFFFu;
    }

    /* 先关 SysTick */
    SYST_CSR = 0u;

    /* 设置 reload */
    SYST_RVR = reload;

    /* 清当前计数 */
    SYST_CVR = 0u;

    /* 使能：时钟源=CPU(HCLK)，开中断，开计数 */
    SYST_CSR = (SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE);
}

/* ======================= 对外 API ======================= */
void Delay_Init(uint32_t sysclk_hz)
{
    g_sysclk_hz = sysclk_hz;
    g_ms_tick = 0u;

    delay_systick_config_1ms(sysclk_hz);
}

/* SysTick_Handler 里调用它（你的 main.c 已经在 ISR 里调用了） */
void Delay_IncTick(void)
{
    g_ms_tick++;
}

uint32_t Delay_GetTick(void)
{
    return g_ms_tick;
}

void Delay_Ms(uint32_t ms)
{
    uint32_t start;

    start = g_ms_tick;
    while ((uint32_t)(g_ms_tick - start) < ms)
    {
        /* wait */
    }
}

/* 简单 us 忙等：后续你要更准，我再按 Keil 优化等级帮你校准系数 */
void Delay_Us(uint32_t us)
{
    uint32_t cycles_per_us;
    uint32_t loops;

    if (g_sysclk_hz == 0u)
    {
        /* 未初始化就兜底做一点 NOP */
        while (us--)
        {
            delay_nop();
        }
        return;
    }

    cycles_per_us = (g_sysclk_hz / 1000000u);
    if (cycles_per_us == 0u)
    {
        cycles_per_us = 1u;
    }

    /* 经验系数：每次循环约消耗若干 cycle（不同优化等级会变）
       先给保守值，保证不会太短 */
    loops = (cycles_per_us * us) / 4u;
    if (loops == 0u)
    {
        loops = 1u;
    }

    while (loops--)
    {
        delay_nop();
    }
}
