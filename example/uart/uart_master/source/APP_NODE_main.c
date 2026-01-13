/******************************************************************************
 * APP_NODE_main.c - PAN3029 Raw RF Node
 *
 * 行为：
 *  - 上电后初始化 RF -> 进入 RX
 *  - 每隔一段时间发送 "PING" 帧（计数递增）
 *  - 发送后进入单次超时 RX，等待协调器返回 "ACK"
 *  - 打印 TX_DONE / RX_DONE / RX_TIMEOUT
 *
 * 串口命令：
 *  h/? : help
 *  i   : print current RF parameters
 *  t   : send one PING now
 *  r   : force enter RX (continuous)
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dbg.h"
#include "pan3029_rf.h"

extern volatile uint32_t g_ms;

/* =================== RF 参数（两块板必须一致） =================== */
/* 你可以先用你 STM32 通的那套参数：频点/BW/SF/CR/SYNC/CRC/TXPOWER */
#ifndef APP_RF_FREQ_HZ
#define APP_RF_FREQ_HZ        (470000000UL)   /* 示例：470MHz，按你实际通的改 */
#endif
#ifndef APP_RF_BW
#define APP_RF_BW             (BW_125K)
#endif
#ifndef APP_RF_SF
#define APP_RF_SF             (SF_7)
#endif
#ifndef APP_RF_CR
#define APP_RF_CR             (CODE_RATE_45)
#endif
#ifndef APP_RF_SYNC
#define APP_RF_SYNC           (0x34u)
#endif
#ifndef APP_RF_CRC_ON
#define APP_RF_CRC_ON         (CRC_ON)
#endif
#ifndef APP_RF_TXPWR
#define APP_RF_TXPWR          (10u)              /* 0..??：按你的 rf_set_tx_power 支持范围 */
#endif

/* =================== 业务参数 =================== */
#define NODE_PING_PERIOD_MS   (2000u)
#define NODE_RX_TIMEOUT_MS    (300u)

static uint32_t s_next_ping_ms = 0u;
static uint8_t  s_seq = 0u;
static uint32_t s_tx_ts = 0u;

static void node_print_help(void)
{
    dbg_puts("\r\n============= NODE(RF) HELP =============\r\n");
    dbg_puts("h/? : help\r\n");
    dbg_puts("i   : print RF parameters\r\n");
    dbg_puts("t   : send one PING now\r\n");
    dbg_puts("r   : force enter RX (continuous)\r\n");
    dbg_puts("========================================\r\n");
}

static void node_print_rf_info(void)
{
    dbg_puts("[NODE][RF] freq=");
    dbg_put_u32((uint32_t)rf_read_freq());
    dbg_puts(" bw=");
    dbg_put_u32((uint32_t)rf_get_bw());
    dbg_puts(" sf=");
    dbg_put_u32((uint32_t)rf_get_sf());
    dbg_puts(" cr=");
    dbg_put_u32((uint32_t)rf_get_code_rate());
    dbg_puts(" crc=");
    dbg_put_u32((uint32_t)rf_get_crc());
    dbg_puts(" sync=0x");
    dbg_put_hex8(rf_get_syncword());
    dbg_puts(" txpwr=");
    dbg_put_u32((uint32_t)rf_get_tx_power());
    dbg_puts(" mode=");
    dbg_put_u32((uint32_t)rf_get_mode());
    dbg_puts("\r\n");
}

static void node_enter_rx_continuous(void)
{
    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_enter_continous_rx();

    dbg_puts("[NODE][RF] enter RX continuous\r\n");
}

static void node_enter_rx_timeout(uint32_t timeout_ms)
{
    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_enter_single_timeout_rx(timeout_ms);
}

static void node_send_ping(void)
{
    uint8_t buf[32];
    uint8_t len = 0u;

    s_seq++;
    s_tx_ts = (uint32_t)g_ms;

    /* 帧格式：'P' 'I' 'N' 'G' seq time32 */
    buf[len++] = 'P';
    buf[len++] = 'I';
    buf[len++] = 'N';
    buf[len++] = 'G';
    buf[len++] = s_seq;
    buf[len++] = (uint8_t)(s_tx_ts & 0xFFu);
    buf[len++] = (uint8_t)((s_tx_ts >> 8) & 0xFFu);
    buf[len++] = (uint8_t)((s_tx_ts >> 16) & 0xFFu);
    buf[len++] = (uint8_t)((s_tx_ts >> 24) & 0xFFu);

    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);

    dbg_puts("[NODE][TX] PING seq=0x");
    dbg_put_hex8(s_seq);
    dbg_puts(" len=");
    dbg_put_u32(len);
    dbg_puts("\r\n");

    (void)rf_send_packet(buf, (int)len);
    /* 发送完成 IRQ 会置 TXDONE；这里不 busy wait，交给 app_task 处理 */

    /* 发送后进入等待 ACK 的单次超时 RX */
    node_enter_rx_timeout(NODE_RX_TIMEOUT_MS);
}

static void node_handle_radio_events(void)
{
    int txf = rf_get_transmit_flag();
    int rxf = rf_get_recv_flag();

    if (txf == RADIO_FLAG_TXDONE)
    {
        (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
        dbg_puts("[NODE][IRQ] TX_DONE\r\n");
    }

    if (rxf == RADIO_FLAG_RXDONE)
    {
        uint8_t rx[128];
        uint8_t n;

        (void)rf_set_recv_flag(RADIO_FLAG_IDLE);

        n = rf_recv_packet(rx);

        dbg_puts("[NODE][RX] len=");
        dbg_put_u32((uint32_t)n);
        dbg_puts(" data=");
        dbg_hexdump(rx, n);
        dbg_puts("\r\n");

        /* 简单判断 ACK */
        if (n >= 4u && rx[0] == 'A' && rx[1] == 'C' && rx[2] == 'K')
        {
            uint8_t ack_seq = rx[3];
            dbg_puts("[NODE] got ACK seq=0x");
            dbg_put_hex8(ack_seq);
            dbg_puts(" rtt(ms)=");
            dbg_put_u32((uint32_t)(g_ms - s_tx_ts));
            dbg_puts("\r\n");
        }

        node_enter_rx_continuous();
    }
    else if (rxf == RADIO_FLAG_RXTIMEOUT)
    {
        (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
        dbg_puts("[NODE][IRQ] RX_TIMEOUT\r\n");
        node_enter_rx_continuous();
    }
    else if (rxf == RADIO_FLAG_RXERR)
    {
        (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
        dbg_puts("[NODE][IRQ] RX_CRC_ERR\r\n");
        node_enter_rx_continuous();
    }
}

static RF_Err_t node_rf_setup(void)
{
    RF_Err_t ret;

    dbg_puts("[NODE][RF] rf_init...\r\n");
    ret = rf_init();
    if (ret != RF_OK && ret != OK)
    {
        dbg_puts("[NODE][RF] rf_init FAIL\r\n");
        return RF_FAIL;
    }

    /* 使用参考工程默认参数（如果你的 STM32 工程里有固定配置，也可以不调用） */
    (void)rf_set_default_para();

    (void)rf_set_freq(APP_RF_FREQ_HZ);
    (void)rf_set_bw(APP_RF_BW);
    (void)rf_set_sf(APP_RF_SF);
    (void)rf_set_code_rate(APP_RF_CR);
    (void)rf_set_syncword(APP_RF_SYNC);
    (void)rf_set_crc((APP_RF_CRC_ON == CRC_ON) ? true : false);
    (void)rf_set_tx_power(APP_RF_TXPWR);

    /* 先清空 IRQ/flag，进入常开 RX */
    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_clr_irq(0xFFu);

    node_print_rf_info();
    node_enter_rx_continuous();
    return OK;
}

void app_node_init(void)
{
    dbg_puts("\r\n[APP] ROLE=NODE (RAW RF)\r\n");
    node_print_help();

    (void)node_rf_setup();

    s_next_ping_ms = (uint32_t)g_ms + 1000u;
}

void app_node_task(void)
{
    node_handle_radio_events();

    if ((uint32_t)g_ms >= s_next_ping_ms)
    {
        node_send_ping();
        s_next_ping_ms = (uint32_t)g_ms + NODE_PING_PERIOD_MS;
    }
}

void app_node_on_cmd(uint8_t ch)
{
    if ((ch == (uint8_t)'h') || (ch == (uint8_t)'?'))
    {
        node_print_help();
    }
    else if (ch == (uint8_t)'i')
    {
        node_print_rf_info();
    }
    else if (ch == (uint8_t)'t')
    {
        node_send_ping();
        s_next_ping_ms = (uint32_t)g_ms + NODE_PING_PERIOD_MS;
    }
    else if (ch == (uint8_t)'r')
    {
        node_enter_rx_continuous();
    }
    else
    {
        dbg_puts("[NODE] unknown cmd: 0x");
        dbg_put_hex8(ch);
        dbg_puts("\r\n");
    }
}
