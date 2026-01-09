#include "ddl.h"
#include "clk.h"
#include "gpio.h"
#include "spi.h"

#include "dbg.h"
#include "pan3029_spi_test.h"

/* ====================== 你可以按硬件实际修改这里 ======================
 * 下面 pinmux 默认沿用你工程里已有的 GPIO->SPI 映射函数。
 * 如果你的 CS/MISO/MOSI/SCK 接在别的引脚，只需要改这些宏即可。
 */
#ifndef PAN_SPI_CS_FUNC
#define PAN_SPI_CS_FUNC()      Gpio_SetFunc_SPI_CS_P03()
#endif

#ifndef PAN_SPI_MISO_FUNC
#define PAN_SPI_MISO_FUNC()    Gpio_SetFunc_SPIMISO_P23()
#endif

#ifndef PAN_SPI_MOSI_FUNC
#define PAN_SPI_MOSI_FUNC()    Gpio_SetFunc_SPIMOSI_P24()
#endif

#ifndef PAN_SPI_SCK_FUNC
#define PAN_SPI_SCK_FUNC()     Gpio_SetFunc_SPICLK_P25()
#endif

/* PAN3029 NRST：默认 P26 (Port2 Pin6) */
#ifndef PAN_RST_PORT
#define PAN_RST_PORT           (2u)
#endif
#ifndef PAN_RST_PIN
#define PAN_RST_PIN            (6u)
#endif

/* SPI 分频：按你板子实际 PCLK 选择。
 * 若 PCLK=32MHz：SpiClkDiv32 -> SCK≈1MHz，抓波形/联通性测试比较稳。
 */
#ifndef PAN_SPI_BAUD
#define PAN_SPI_BAUD           (SpiClkDiv32)
#endif
/* ===================================================================== */

/* --------- PAN3029 寄存器（手册 12.1/12.2） ---------
 * 0x00: REG_SOFT_RST(bit7) + REG_PAGE_SEL(bit1:0)
 * 0x02: REG_OPERATE_MODE(bit2:0)
 * 0x04: EN_LS_3V(bit5) + POR_RSTL(bit4)
 */
#define PAN_REG_SYS_PAGE        (0x00u)
#define PAN_REG_OPERATE_MODE    (0x02u)
#define PAN_REG_LS_3V           (0x04u)

#define PAN_PAGE_MASK           (0x03u)

/* Address Byte：addr[6:0] + wr[0]（wr=1写，wr=0读） */
static uint8_t pan_cmd_read(uint8_t addr)  { return (uint8_t)((addr << 1) | 0u); }
static uint8_t pan_cmd_write(uint8_t addr) { return (uint8_t)((addr << 1) | 1u); }

static uint8_t pan_spi_rw(uint8_t tx)
{
    uint32_t timeout = 20000u;

    M0P_SPI->DATA = tx;

    while (timeout--)
    {
        if (TRUE == Spi_GetStatus(SpiIf))
        {
            break;
        }
    }

    if (timeout == 0u)
    {
        return 0u;
    }

    return (uint8_t)M0P_SPI->DATA;
}

void pan3029_hw_reset(void)
{
    Gpio_SetIO(PAN_RST_PORT, PAN_RST_PIN, 0);
    delay1ms(5);
    Gpio_SetIO(PAN_RST_PORT, PAN_RST_PIN, 1);
    delay1ms(10);
}

void pan3029_spi_test_init(void)
{
    stc_spi_config_t spiCfg;

    /* SPI 外设门控 */
    Clk_SetPeripheralGate(ClkPeripheralSpi, TRUE);

    /* PinMux */
    PAN_SPI_CS_FUNC();
    PAN_SPI_MISO_FUNC();
    PAN_SPI_MOSI_FUNC();
    PAN_SPI_SCK_FUNC();

    /* NRST */
    Gpio_InitIO(PAN_RST_PORT, PAN_RST_PIN, GpioDirOut);
    Gpio_SetIO(PAN_RST_PORT, PAN_RST_PIN, 1);

    /* 先拉高 CSN（SSN=1 表示不选中） */
    Spi_SetCS(TRUE);

    /* SPI Mode0：CPOL=0, CPHA=0
     * 你的 DDL 里：Spicpollow + Spicphafirst
     */
    spiCfg.bCPHA       = Spicphafirst;
    spiCfg.bCPOL       = Spicpollow;
    spiCfg.bIrqEn      = FALSE;
    spiCfg.bMasterMode = SpiMaster;
    spiCfg.u8BaudRate  = PAN_SPI_BAUD;
    spiCfg.pfnIrqCb    = NULL;

    (void)Spi_Init(&spiCfg);

    pan3029_hw_reset();
}

uint8_t pan3029_read_reg(uint8_t addr)
{
    uint8_t cmd = pan_cmd_read(addr);
    uint8_t val;

    Spi_SetCS(FALSE);
    (void)pan_spi_rw(cmd);
    val = pan_spi_rw(0x00u);
    Spi_SetCS(TRUE);

    return val;
}

void pan3029_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t cmd = pan_cmd_write(addr);

    Spi_SetCS(FALSE);
    (void)pan_spi_rw(cmd);
    (void)pan_spi_rw(val);
    Spi_SetCS(TRUE);
}

void pan3029_read_burst(uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t cmd = pan_cmd_read(addr);

    if ((buf == (uint8_t *)0) || (len == 0u))
    {
        return;
    }

    Spi_SetCS(FALSE);
    (void)pan_spi_rw(cmd);

    for (i = 0; i < len; i++)
    {
        buf[i] = pan_spi_rw(0x00u);
    }

    Spi_SetCS(TRUE);
}

void pan3029_write_burst(uint8_t addr, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t cmd = pan_cmd_write(addr);

    if ((buf == (const uint8_t *)0) || (len == 0u))
    {
        return;
    }

    Spi_SetCS(FALSE);
    (void)pan_spi_rw(cmd);

    for (i = 0; i < len; i++)
    {
        (void)pan_spi_rw(buf[i]);
    }

    Spi_SetCS(TRUE);
}

static void pan_print_reg(const char *name, uint8_t addr)
{
    uint8_t v = pan3029_read_reg(addr);

    dbg_puts("[SPI] ");
    dbg_puts(name);
    dbg_puts("(0x");
    dbg_put_hex8(addr);
    dbg_puts(")=0x");
    dbg_put_hex8(v);
    dbg_puts("\r\n");
}

static bool pan_set_page(uint8_t page, bool verbose)
{
    uint8_t r0;
    uint8_t w0;
    uint8_t rb;

    r0 = pan3029_read_reg(PAN_REG_SYS_PAGE);
    w0 = (uint8_t)((r0 & (uint8_t)(~PAN_PAGE_MASK)) | (page & PAN_PAGE_MASK));

    pan3029_write_reg(PAN_REG_SYS_PAGE, w0);
    rb = pan3029_read_reg(PAN_REG_SYS_PAGE);

    if (verbose)
    {
        dbg_puts("[SPI] set page ");
        dbg_put_u32((uint32_t)page);
        dbg_puts(": write 0x");
        dbg_put_hex8(w0);
        dbg_puts(" -> readback 0x");
        dbg_put_hex8(rb);
        dbg_puts("\r\n");
    }

    /* 只验证 page[1:0] 是否等于期望值（bit7 软复位不在这里动） */
    if ((rb & PAN_PAGE_MASK) != (page & PAN_PAGE_MASK))
    {
        if (verbose)
        {
            dbg_puts("[SPI] FAIL: page readback mismatch!\r\n");
        }
        return false;
    }

    return true;
}

bool pan3029_spi_sanity_test(bool verbose)
{
    uint8_t r0, r2, r4;
    uint8_t saved_r0;
    bool ok = true;

    if (verbose)
    {
        dbg_puts("\r\n========== PAN3029 SPI SANITY TEST ==========\r\n");
        dbg_puts("Test idea: write/read REG_PAGE_SEL in reg 0x00 (W/R)\r\n");
    }

    /* 1) 先读几个“看起来不该全0/全FF”的寄存器 */
    r0 = pan3029_read_reg(PAN_REG_SYS_PAGE);
    r2 = pan3029_read_reg(PAN_REG_OPERATE_MODE);
    r4 = pan3029_read_reg(PAN_REG_LS_3V);

    if (verbose)
    {
        pan_print_reg("REG_SYS_PAGE", PAN_REG_SYS_PAGE);
        pan_print_reg("REG_OPERATE_MODE", PAN_REG_OPERATE_MODE);
        pan_print_reg("REG_LS_3V", PAN_REG_LS_3V);
    }

    /* 如果全部 0x00 或全部 0xFF，通常是 SPI 没通（MISO浮/CS不对/时序不对） */
    if ((r0 == 0x00u && r2 == 0x00u && r4 == 0x00u) ||
        (r0 == 0xFFu && r2 == 0xFFu && r4 == 0xFFu))
    {
        if (verbose)
        {
            dbg_puts("[SPI] FAIL: R0/R2/R4 all 0x00 or all 0xFF -> check CS/SCK/MISO/Mode\r\n");
            dbg_puts("===========================================\r\n");
        }
        return false;
    }

    if ((r0 == r2) && (r2 == r4) && verbose)
    {
        dbg_puts("[SPI] WARN: R0/R2/R4 are same. Not always wrong, but suspicious.\r\n");
    }

    /* 2) page 切换回读验证：0->1->2->3->恢复 */
    saved_r0 = r0;

    if (verbose)
    {
        dbg_puts("[SPI] Page switch readback test...\r\n");
    }

    ok &= pan_set_page(0u, verbose);
    ok &= pan_set_page(1u, verbose);
    ok &= pan_set_page(2u, verbose);
    ok &= pan_set_page(3u, verbose);

    /* 恢复原 page */
    pan3029_write_reg(PAN_REG_SYS_PAGE, saved_r0);

    if (verbose)
    {
        dbg_puts("[SPI] restore r0=0x");
        dbg_put_hex8(saved_r0);
        dbg_puts("\r\n");

        dbg_puts(ok ? "[SPI] PASS: SPI read/write is OK!\r\n"
                    : "[SPI] FAIL: SPI read/write NOT OK!\r\n");
        dbg_puts("===========================================\r\n");
    }

    return ok;
}

/* ====================== CLI ======================
 * h  : help
 * i  : init + reset + sanity_test
 * x  : hw_reset
 * rAA: read  reg AA
 * wAAVV: write reg AA = VV
 * dAANN: dump from AA len NN
 * pN : set page N (0~3)  ——内部写 0x00 的 page bit
 * t  : run sanity_test
 * ================================================ */

static uint8_t hex_val(uint8_t c, bool *ok)
{
    if (c >= '0' && c <= '9') { *ok = true; return (uint8_t)(c - '0'); }
    if (c >= 'a' && c <= 'f') { *ok = true; return (uint8_t)(c - 'a' + 10); }
    if (c >= 'A' && c <= 'F') { *ok = true; return (uint8_t)(c - 'A' + 10); }
    *ok = false;
    return 0u;
}

static bool read_hex_byte_blocking(uint8_t *out)
{
    uint8_t c;
    uint8_t hi, lo;
    bool ok;

    while (!dbg_getch_nonblock(&c)) {}
    hi = hex_val(c, &ok); if (!ok) return false;

    while (!dbg_getch_nonblock(&c)) {}
    lo = hex_val(c, &ok); if (!ok) return false;

    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static bool read_dec_nibble_blocking(uint8_t *out) /* 读一个 '0'~'9' 字符 */
{
    uint8_t c;
    while (!dbg_getch_nonblock(&c)) {}
    if (c < '0' || c > '9') return false;
    *out = (uint8_t)(c - '0');
    return true;
}

static void cli_help(void)
{
    dbg_puts("\r\n==== PAN3029 SPI CLI ====\r\n");
    dbg_puts("h            : help\r\n");
    dbg_puts("i            : init + hw_reset + sanity_test\r\n");
    dbg_puts("x            : hw_reset\r\n");
    dbg_puts("t            : run sanity_test\r\n");
    dbg_puts("pN           : set page N (0~3)\r\n");
    dbg_puts("rAA          : read  reg AA\r\n");
    dbg_puts("wAAVV        : write reg AA=VV\r\n");
    dbg_puts("dAANN        : dump from AA, len NN (max 64)\r\n");
    dbg_puts("=========================\r\n");
}

void pan3029_spi_cli_poll(void)
{
    uint8_t c;
    uint8_t addr, val, len;
    uint8_t page;
    uint8_t buf[64];
    uint16_t i;

    if (!dbg_getch_nonblock(&c))
    {
        return;
    }

    if (c == 'h' || c == 'H')
    {
        cli_help();
    }
    else if (c == 'i' || c == 'I')
    {
        pan3029_spi_test_init();
        dbg_puts("[SPI] init OK\r\n");
        (void)pan3029_spi_sanity_test(true);
    }
    else if (c == 'x' || c == 'X')
    {
        pan3029_hw_reset();
        dbg_puts("[SPI] hw_reset OK\r\n");
    }
    else if (c == 't' || c == 'T')
    {
        (void)pan3029_spi_sanity_test(true);
    }
    else if (c == 'p' || c == 'P')
    {
        if (!read_dec_nibble_blocking(&page))
        {
            dbg_puts("[SPI] bad page (0~3)\r\n");
            return;
        }
        if (page > 3u)
        {
            dbg_puts("[SPI] page range 0~3\r\n");
            return;
        }
        if (pan_set_page(page, true))
        {
            dbg_puts("[SPI] page set OK\r\n");
        }
        else
        {
            dbg_puts("[SPI] page set FAIL\r\n");
        }
    }
    else if (c == 'r' || c == 'R')
    {
        if (!read_hex_byte_blocking(&addr))
        {
            dbg_puts("[SPI] bad hex\r\n");
            return;
        }
        val = pan3029_read_reg(addr);
        dbg_puts("[SPI] R 0x"); dbg_put_hex8(addr);
        dbg_puts(" = 0x");      dbg_put_hex8(val);
        dbg_puts("\r\n");
    }
    else if (c == 'w' || c == 'W')
    {
        if (!read_hex_byte_blocking(&addr) || !read_hex_byte_blocking(&val))
        {
            dbg_puts("[SPI] bad hex\r\n");
            return;
        }
        pan3029_write_reg(addr, val);
        dbg_puts("[SPI] W 0x"); dbg_put_hex8(addr);
        dbg_puts(" <= 0x");     dbg_put_hex8(val);
        dbg_puts("\r\n");
    }
    else if (c == 'd' || c == 'D')
    {
        if (!read_hex_byte_blocking(&addr) || !read_hex_byte_blocking(&len))
        {
            dbg_puts("[SPI] bad hex\r\n");
            return;
        }
        if (len > (uint8_t)sizeof(buf)) len = (uint8_t)sizeof(buf);

        pan3029_read_burst(addr, buf, len);

        dbg_puts("[SPI] DUMP @0x"); dbg_put_hex8(addr);
        dbg_puts(" len=0x");        dbg_put_hex8(len);
        dbg_puts("\r\n");

        for (i = 0; i < len; i++)
        {
            dbg_put_hex8(buf[i]);
            dbg_puts(((i & 0x0Fu) == 0x0Fu) ? "\r\n" : " ");
        }
        dbg_puts("\r\n");
    }
    else
    {
        /* ignore */
    }
}
