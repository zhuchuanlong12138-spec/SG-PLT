#ifndef __BOARD_PINS_H__
#define __BOARD_PINS_H__

/*
 * board_pins.h
 * ------------------------------------------------------------
 * HC32L110C6PA (TSSOP20) 板级引脚/外设统一配置
 *
 * 本版用于：PAN3029 + DL2807 MAC
 * SPI 引脚：完全对齐 HC32 官方 SPI 例程（强烈建议你按此修改硬件连线）
 *
 * 连接（官方 SPI 例程引脚）：
 *   PAN3029  SCK  -> HC32 P25 (port=2 pin=5)
 *   PAN3029  MOSI -> HC32 P24 (port=2 pin=4)
 *   PAN3029  MISO -> HC32 P23 (port=2 pin=3)
 *   PAN3029  CSN  -> HC32 P03 (port=0 pin=3) 低有效（由 SPI.SSN 控制，Spi_SetCS）
 *
 * 其余 GPIO（你可按硬件实际改）：
 *   PAN3029  IRQ  -> HC32 P15 (port=1 pin=5)  （建议避开SPI脚）
 *   PAN3029  NRST -> HC32 P26 (port=2 pin=6)  低有效
 *   PAN3029  GPIO11(CAD) -> HC32 P27 (port=2 pin=7)
 */

#include <stdint.h>
#include "ddl.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_PORT_A      (0u)   /* P0 */
#define BOARD_PORT_B      (1u)   /* P1 */
#define BOARD_PORT_C      (2u)   /* P2 */
#define BOARD_PORT_D      (3u)   /* P3 */

#define BOARD_PIN(n)      ((uint8_t)(n))

/* ===================== PAN3029 - SPI（官方例程引脚） ===================== */
/* SPI SCK  -> P25 */
#define PAN3029_PIN_SCK_PORT      (BOARD_PORT_C)
#define PAN3029_PIN_SCK_PIN       (BOARD_PIN(5))     /* P25 */

/* SPI MOSI -> P24 */
#define PAN3029_PIN_MOSI_PORT     (BOARD_PORT_C)
#define PAN3029_PIN_MOSI_PIN      (BOARD_PIN(4))     /* P24 */

/* SPI MISO -> P23 */
#define PAN3029_PIN_MISO_PORT     (BOARD_PORT_C)
#define PAN3029_PIN_MISO_PIN      (BOARD_PIN(3))     /* P23 */

/* SPI CSN  -> P03（由 SPI.SSN 控制，不走 GPIO 拉高拉低） */
#define PAN3029_PIN_CS_PORT       (BOARD_PORT_A)
#define PAN3029_PIN_CS_PIN        (BOARD_PIN(3))     /* P03 */

/* ===================== PAN3029 - IRQ / RST / CAD ===================== */
/* IRQ：建议用 P15（避免占用 SPI 引脚） */
#define PAN3029_PIN_IRQ_PORT      (BOARD_PORT_B)
#define PAN3029_PIN_IRQ_PIN       (BOARD_PIN(5))     /* P15 */

/* RST（NRST，低有效） */
#define PAN3029_RST_PRESENT       (1u)
#define PAN3029_PIN_RST_PORT      (BOARD_PORT_C)
#define PAN3029_PIN_RST_PIN       (BOARD_PIN(6))     /* P26 */

/* CAD（GPIO11 输出） */
#define PAN3029_CAD_PRESENT       (1u)
#define PAN3029_PIN_CAD_PORT      (BOARD_PORT_C)
#define PAN3029_PIN_CAD_PIN       (BOARD_PIN(7))     /* P27 */

/* ===================== 兼容旧宏名（供老代码引用） ===================== */
#ifndef PAN3029_CS_PORT
#define PAN3029_CS_PORT           (PAN3029_PIN_CS_PORT)
#endif
#ifndef PAN3029_CS_PIN
#define PAN3029_CS_PIN            (PAN3029_PIN_CS_PIN)
#endif

#ifndef PAN3029_IRQ_PORT
#define PAN3029_IRQ_PORT          (PAN3029_PIN_IRQ_PORT)
#endif
#ifndef PAN3029_IRQ_PIN
#define PAN3029_IRQ_PIN           (PAN3029_PIN_IRQ_PIN)
#endif

#ifndef PAN3029_RST_PORT
#define PAN3029_RST_PORT          (PAN3029_PIN_RST_PORT)
#endif
#ifndef PAN3029_RST_PIN
#define PAN3029_RST_PIN           (PAN3029_PIN_RST_PIN)
#endif

#ifndef PAN3029_IRQ_READ
#define PAN3029_IRQ_READ()        (Gpio_GetIO(PAN3029_IRQ_PORT, PAN3029_IRQ_PIN))
#endif

#if (PAN3029_CAD_PRESENT == 1u)
  #ifndef PAN3029_CAD_READ
  #define PAN3029_CAD_READ()      (Gpio_GetIO(PAN3029_PIN_CAD_PORT, PAN3029_PIN_CAD_PIN))
  #endif
#else
  #ifndef PAN3029_CAD_READ
  #define PAN3029_CAD_READ()      (0u)
  #endif
#endif

#if (PAN3029_RST_PRESENT == 1u)
  #ifndef PAN3029_RST_LOW
  #define PAN3029_RST_LOW()       Gpio_SetIO(PAN3029_RST_PORT, PAN3029_RST_PIN, FALSE)
  #endif
  #ifndef PAN3029_RST_HIGH
  #define PAN3029_RST_HIGH()      Gpio_SetIO(PAN3029_RST_PORT, PAN3029_RST_PIN, TRUE)
  #endif
#else
  #ifndef PAN3029_RST_LOW
  #define PAN3029_RST_LOW()       do{}while(0)
  #endif
  #ifndef PAN3029_RST_HIGH
  #define PAN3029_RST_HIGH()      do{}while(0)
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_PINS_H__ */
