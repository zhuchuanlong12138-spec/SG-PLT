#include "pan3029_port.h"
#include "board_pins.h"
#include "delay.h"
#include "spi.h"
#include <stdint.h>

/* ????? dbg.c ? */
extern void dbg_puts(const char *s);
extern void dbg_put_hex8(uint8_t v);
extern void dbg_put_u32(uint32_t v);
extern volatile uint32_t g_ms;

/* rf ? pan3029_rf.c ? */
extern uint8_t rf_read_reg(uint8_t addr);
extern void rf_irq_process(void);

/* ????? */
#define LOGS(s)          do { dbg_puts(s); } while (0)
#define LOG_HEX8(v)      do { dbg_put_hex8((uint8_t)(v)); } while (0)

static uint8_t s_hw_inited = 0u;

static void log_prefix(const char *tag)
{
    dbg_puts("\r\n[");
    dbg_puts(tag);
    dbg_puts("] ms=");
    dbg_put_u32((uint32_t)g_ms);
    dbg_puts(" ");
}

/* ===================== SPI ????? ===================== */
static void pan3029_spi_init_official(void)
{
    stc_spi_config_t cfg;

    DDL_ZERO_STRUCT(cfg);

    /* SPI PinMux：与已验证可用的 SPI 测试工程保持一致 */
    Gpio_SetFunc_SPI_CS_P03();
    Gpio_SetFunc_SPIMISO_P23();
    Gpio_SetFunc_SPIMOSI_P24();
    Gpio_SetFunc_SPICLK_P25();

    /* SPI 外设门控 */
    Clk_SetPeripheralGate(ClkPeripheralSpi, TRUE);

    /* 先拉高 CSN（SSN=1 表示不选中，从机释放 MISO） */
    Spi_SetCS(TRUE);

    /* SPI Mode0：CPOL=0, CPHA=0；Master；关闭中断 */
    cfg.bCPHA       = Spicphafirst;
    cfg.bCPOL       = Spicpollow;
    cfg.bIrqEn      = FALSE;
    cfg.bMasterMode = SpiMaster;
    cfg.u8BaudRate  = SpiClkDiv32; /* 建议先慢一点，稳定后再提速 */
    cfg.pfnIrqCb    = NULL;

    (void)Spi_Init(&cfg);

    log_prefix("SPI");
    LOGS("init ok (Mode0, Master, SSN=P03, Div32)");
}


/* ===================== GPIO ?IRQ/RST/CAD ===================== */
static void pan3029_gpio_init_once(void)
{
    if (s_hw_inited != 0u)
    {
        return;
    }

    log_prefix("RF");
    LOGS("gpio init...");

    /* RST  */
#if (PAN3029_RST_PRESENT == 1u)
    (void)Gpio_InitIO(PAN3029_RST_PORT, PAN3029_RST_PIN, GpioDirOut);
    PAN3029_RST_HIGH();
#endif

    /* IRQ  + ж? */
    (void)Gpio_InitIO(PAN3029_IRQ_PORT, PAN3029_IRQ_PIN, GpioDirIn);
    (void)Gpio_EnableIrq(PAN3029_IRQ_PORT, PAN3029_IRQ_PIN, PAN3029_IRQ_TRIG);

    /* CAD ?? */
#if (PAN3029_CAD_PRESENT == 1u)
    (void)Gpio_InitIO(PAN3029_PIN_CAD_PORT, PAN3029_PIN_CAD_PIN, GpioDirIn);
#endif

#ifdef PORT2_IRQn
    EnableNvic(PORT2_IRQn, 3u, TRUE);
#endif

    /* SPI ??/迪? */
    pan3029_spi_init_official();

    s_hw_inited = 1u;

    log_prefix("RF");
    LOGS("gpio init ok");
}

static void pan3029_hw_reset(void)
{
#if (PAN3029_RST_PRESENT == 1u)
    log_prefix("RF");
    LOGS("reset low 5ms -> high 10ms");

    PAN3029_RST_LOW();
    Delay_Ms(5u);
    PAN3029_RST_HIGH();
    Delay_Ms(10u);

    log_prefix("RF");
    LOGS("reset done");
#endif
}

/* ===================== rf_port ?壨 pan3029_rf.c ?? ===================== */
rf_port_t rf_port =
{
    rf_antenna_init,
    rf_tcxo_init,
    rf_antenna_tx,
    rf_antenna_rx,
    rf_antenna_close,
    rf_tcxo_close,
    spi_readwritebyte,
    spi_cs_set_high,
    spi_cs_set_low,
    rf_delay_ms,
    rf_delay_us,
};

/* ===================== SPI/CS/Delay ? ===================== */
uint8_t spi_readwritebyte(uint8_t tx_data)
{
    uint32_t timeout;
    uint8_t  rx;

    timeout = 20000u;

    M0P_SPI->DATA = tx_data;

    while (timeout--)
    {
        if (TRUE == Spi_GetStatus(SpiIf))
        {
            break;
        }
    }

    if (timeout == 0u)
    {
        log_prefix("SPI");
        LOGS("timeout tx=0x");
        LOG_HEX8(tx_data);
        return 0u;
    }

    rx = (uint8_t)M0P_SPI->DATA;
    return rx;
}

/* CSN ЧSpi_SetCS(FALSE)=?TRUE= */
void spi_cs_set_high(void)
{
    Spi_SetCS(TRUE);
}

void spi_cs_set_low(void)
{
    Spi_SetCS(FALSE);
}

void rf_delay_ms(uint32_t time)
{
    Delay_Ms(time);
}

void rf_delay_us(uint32_t time)
{
    Delay_Us(time);
}

/* ===================== antenna/tcxo + ? ===================== */
void rf_antenna_init(void)
{
    uint8_t r0;
    uint8_t r2;
    uint8_t r4;

    pan3029_gpio_init_once();
    pan3029_hw_reset();

    log_prefix("RF");
    LOGS("probe regs... (expect not all 00/FF)");

    r0 = rf_read_reg(0x00u);
    r2 = rf_read_reg(0x02u);
    r4 = rf_read_reg(0x04u);

    log_prefix("RF");
    LOGS("REG00=0x"); LOG_HEX8(r0);
    LOGS(" REG02=0x"); LOG_HEX8(r2);
    LOGS(" REG04=0x"); LOG_HEX8(r4);

    log_prefix("RF");
    LOGS("IRQ="); LOG_HEX8((uint8_t)(PAN3029_IRQ_READ() ? 1u : 0u));
#if (PAN3029_CAD_PRESENT == 1u)
    LOGS(" CAD="); LOG_HEX8((uint8_t)(PAN3029_CAD_READ() ? 1u : 0u));
#endif
}

void rf_tcxo_init(void)     { }
void rf_tcxo_close(void)    { }
void rf_antenna_rx(void)    { }
void rf_antenna_tx(void)    { }
void rf_antenna_close(void) { }

/* ===================== GPIO ж?IRQ -> rf_irq_process() ===================== */
void Gpio_IRQHandler(uint8_t u8Port)
{
    if (u8Port == PAN3029_IRQ_PORT)
    {
        if (TRUE == Gpio_GetIrqStat(PAN3029_IRQ_PORT, PAN3029_IRQ_PIN))
        {
            (void)Gpio_ClearIrq(PAN3029_IRQ_PORT, PAN3029_IRQ_PIN);

            log_prefix("IRQ");
            LOGS("PAN3029 IRQ edge");

            rf_irq_process();
        }
    }
}


/* ===================== Pretest helpers ===================== */
void pan3029_port_init_once(void)
{
    pan3029_gpio_init_once();
}

void pan3029_port_hw_reset(void)
{
    pan3029_hw_reset();
}
