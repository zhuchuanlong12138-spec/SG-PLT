#ifndef __DELAY_H__
#define __DELAY_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：配置 SysTick 1ms */
void Delay_Init(uint32_t sysclk_hz);

/* ISR 中调用：每 1ms 递增 tick */
void Delay_IncTick(void);

/* 获取 ms tick（可选） */
uint32_t Delay_GetTick(void);

/* 阻塞延时 */
void Delay_Ms(uint32_t ms);
void Delay_Us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H__ */
