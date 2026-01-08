/*******************************************************************************
 * @file    pan3029_port.h
 * @brief   PAN3029 porting layer for HC32L110C6PA (Keil/armcc v5)
 *
 * 硬件连线（已确认）：
 *   PAN3029 SCK        -> HC32 P15
 *   PAN3029 MOSI       -> HC32 P14
 *   PAN3029 MISO       -> HC32 P23
 *   PAN3029 CS         -> HC32 P24
 *   PAN3029 IRQ        -> HC32 P25
 *   PAN3029 NRST       -> HC32 P26
 *   PAN3029 GPIO11(CAD)-> HC32 P27   (GPIO11 复用输出 CAD)
 *
 * 说明：
 *  - 你的硬件没有外置天线开关(TX/RX)与 TCXO_EN，引脚相关接口在本 port 层做空实现。
 *  - 为了避免循环 include：本文件不再包含 pan3029_rf.h。
 *******************************************************************************/
#ifndef __PAN3029_PORT_H_
#define __PAN3029_PORT_H_

#include <stdint.h>
#include <stdbool.h>

/* HC32 底层驱动 */
#include "gpio.h"
#include "spi.h"
#include "interrupts_hc32l110.h"
#include "delay.h"

/* 板级引脚定义（IRQ/CAD/CS/RST） */
#include "board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================
 *  HC32 引脚映射（Pxy => Port=x Pin=y）
 *  - SPI 三线通常由 SPI 外设接管；CS/RST/IRQ/CAD 由 GPIO 控制/读取
 *==============================*/
#ifndef PAN3029_CS_PORT
#define PAN3029_CS_PORT       (PAN3029_PIN_CS_PORT)
#endif
#ifndef PAN3029_CS_PIN
#define PAN3029_CS_PIN        (PAN3029_PIN_CS_PIN)
#endif

#ifndef PAN3029_IRQ_PORT
#define PAN3029_IRQ_PORT      (PAN3029_PIN_IRQ_PORT)
#endif
#ifndef PAN3029_IRQ_PIN
#define PAN3029_IRQ_PIN       (PAN3029_PIN_IRQ_PIN)
#endif

#ifndef PAN3029_RST_PORT
#define PAN3029_RST_PORT      (PAN3029_PIN_RST_PORT)
#endif
#ifndef PAN3029_RST_PIN
#define PAN3029_RST_PIN       (PAN3029_PIN_RST_PIN)
#endif

#ifndef PAN3029_CAD_PORT
#define PAN3029_CAD_PORT      (PAN3029_PIN_CAD_PORT)
#endif
#ifndef PAN3029_CAD_PIN
#define PAN3029_CAD_PIN       (PAN3029_PIN_CAD_PIN)
#endif

/* IRQ 触发方式：默认上升沿；如发现电平相反/不进中断，可改为 Falling */
#ifndef PAN3029_IRQ_TRIG
#define PAN3029_IRQ_TRIG      (GpioIrqRising)
#endif

/*==============================
 *  rf_port 接口（Panchip 驱动要求）
 *==============================*/
typedef struct
{
    void    (*antenna_init)(void);
    void    (*tcxo_init)(void);
    void    (*set_tx)(void);
    void    (*set_rx)(void);
    void    (*antenna_close)(void);
    void    (*tcxo_close)(void);
    uint8_t (*spi_readwrite)(uint8_t tx_data);
    void    (*spi_cs_high)(void);
    void    (*spi_cs_low)(void);
    void    (*delayms)(uint32_t time);
    void    (*delayus)(uint32_t time);
} rf_port_t;

extern rf_port_t rf_port;

/*==============================
 *  对外函数声明
 *==============================*/
uint8_t spi_readwritebyte(uint8_t tx_data);
void    spi_cs_set_high(void);
void    spi_cs_set_low(void);

void    rf_delay_ms(uint32_t time);
void    rf_delay_us(uint32_t time);

void    rf_antenna_init(void);
void    rf_tcxo_init(void);
void    rf_tcxo_close(void);
void    rf_antenna_rx(void);
void    rf_antenna_tx(void);
void    rf_antenna_close(void);

#ifdef __cplusplus
}
#endif

#endif /* __PAN3029_PORT_H_ */
