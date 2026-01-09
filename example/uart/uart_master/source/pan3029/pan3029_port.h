/*******************************************************************************
 * @file    pan3029_port.h
 * @brief   PAN3029 porting layer for HC32L110C6PA (Keil/armcc v5)
 *
 * ????
 *   PAN3029 SCK        -> HC32 P15
 *   PAN3029 MOSI       -> HC32 P14
 *   PAN3029 MISO       -> HC32 P23
 *   PAN3029 CS         -> HC32 P24
 *   PAN3029 IRQ        -> HC32 P25
 *   PAN3029 NRST       -> HC32 P26
 *   PAN3029 GPIO11(CAD)-> HC32 P27   (GPIO11  CAD)
 *
 * ?
 *  - ???(TX/RX) TCXO_EN??? port ??
 *  - ??? include?? pan3029_rf.h
 *******************************************************************************/
#ifndef __PAN3029_PORT_H_
#define __PAN3029_PORT_H_

#include <stdint.h>
#include <stdbool.h>

/* HC32 ? */
#include "gpio.h"
#include "spi.h"
#include "interrupts_hc32l110.h"
#include "delay.h"

/* 弶?壨IRQ/CAD/CS/RST */
#include "board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================
 *  HC32 ??Pxy => Port=x Pin=y
 *  - SPI ? SPI ??CS/RST/IRQ/CAD  GPIO /?
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

/* IRQ ???緢???/ж??? Falling */
#ifndef PAN3029_IRQ_TRIG
#define PAN3029_IRQ_TRIG      (GpioIrqRising)
#endif

/*==============================
 *  rf_port ??Panchip ?
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
 *  ?
 *==============================*/

/* 供 rf_init 之前做 SPI 预检：仅做一次 GPIO/SPI 初始化 + 硬件复位 */
void pan3029_port_init_once(void);
void pan3029_port_hw_reset(void);

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
