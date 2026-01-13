/******************************************************************************
 * APP_NODE_main.c - PAN3029 Raw RF Node (RX printer)
 *
 * 需求实现：
 *  - COORD 每 1 秒发送一个自增的数（帧头 'C''N''T'+counter32）
 *  - NODE 常开 RX，收到后打印 counter 数值
 *
 * 串口命令：
 *  h/? : help
 *  i   : print current RF parameters
 *  r   : force enter RX (continuous)
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dbg.h"
#include "pan3029_rf.h"

extern volatile uint32_t g_ms;

/* =================== RF 参数（两块板必须一致） =================== */
#ifndef APP_RF_FREQ_HZ
#define APP_RF_FREQ_HZ        (470000000UL)
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
#define APP_RF_TXPWR          (10u)
#endif

static void node_print_help(void)
{
    dbg_puts("\r\n============= NODE(RF) HELP =============\r\n");
    dbg_puts("h/? : help\r\n");
    dbg_puts("i   : print RF parameters\r\n");
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

static void node_handle_radio_events(void)
{
    int rxf = rf_get_recv_flag();

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

        /* 解析 CNT 帧 */
        if (n >= 7u && rx[0] == 'C' && rx[1] == 'N' && rx[2] == 'T')
        {
            uint32_t v = 0u;
            v |= (uint32_t)rx[3];
            v |= ((uint32_t)rx[4] << 8);
            v |= ((uint32_t)rx[5] << 16);
            v |= ((uint32_t)rx[6] << 24);

            dbg_puts("[NODE] CNT=");
            dbg_put_u32(v);
            dbg_puts(" at ms=");
            dbg_put_u32((uint32_t)g_ms);
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

    //(void)rf_set_default_para();

    (void)rf_set_freq(APP_RF_FREQ_HZ);
    (void)rf_set_bw(APP_RF_BW);
    (void)rf_set_sf(APP_RF_SF);
    (void)rf_set_code_rate(APP_RF_CR);
    (void)rf_set_syncword(APP_RF_SYNC);
    (void)rf_set_crc((APP_RF_CRC_ON == CRC_ON) ? true : false);
    (void)rf_set_tx_power(APP_RF_TXPWR);

    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_clr_irq(0xFFu);

    node_print_rf_info();
    node_enter_rx_continuous();
    return OK;
}

void app_node_init(void)
{
    dbg_puts("\r\n[APP] ROLE=NODE (RAW RF, RX printer)\r\n");
    node_print_help();

    (void)node_rf_setup();
}

void app_node_task(void)
{
    node_handle_radio_events();
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
