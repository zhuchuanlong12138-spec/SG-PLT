#ifndef __PAN3029_SPI_TEST_H__
#define __PAN3029_SPI_TEST_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * PAN3029 SPI 最小联通性测试模块（只验证 SPI 读写是否通，不涉及协议/MAC）
 *
 * 手册要点：
 * - CSN 低有效；一次传输：CSN 拉低开始，拉高结束。
 * - Address Byte：前 7bit 为地址 addr；最后 1bit 为读写位 wr（写=1，读=0）。
 * - SCK 下降沿产生数据，上升沿采样数据（等价 SPI Mode0：CPOL=0, CPHA=0）。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：SPI pinmux + SPI 外设 + PAN3029 NRST 复位脚 */
void pan3029_spi_test_init(void);

/* 硬复位：拉低 NRST 一段时间再拉高 */
void pan3029_hw_reset(void);

/* 单字节寄存器读写 */
uint8_t pan3029_read_reg(uint8_t addr);
void    pan3029_write_reg(uint8_t addr, uint8_t val);

/* 连续读写（Burst）；注意：FIFO 模式请使用 addr=0x01（手册 9.4） */
void pan3029_read_burst(uint8_t addr, uint8_t *buf, uint16_t len);
void pan3029_write_burst(uint8_t addr, const uint8_t *buf, uint16_t len);

/*
 * 关键自检：
 * 1) 读取 0x00（REG_PAGE_SEL/REG_SOFT_RST）
 * 2) 仅切换 Page[1:0]（0->1->2->3->恢复），每次写后读回确认
 */
bool pan3029_spi_sanity_test(bool verbose);

/* 串口小交互：放进 while(1) 周期调用即可 */
void pan3029_spi_cli_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __PAN3029_SPI_TEST_H__ */
