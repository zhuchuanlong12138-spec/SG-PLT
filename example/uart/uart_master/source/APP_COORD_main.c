/******************************************************************************
 * APP_COORD_main.c - PAN3029 Raw RF Coordinator
 *
 * 行为：
 *  - 上电后初始化 RF -> 进入 RX continuous
 *  - 收到 PING... 回 ACK(seq)
 *  - 打印 RX_DONE / RX_TIMEOUT / CRC_ERR / TX_DONE
 *
 * 串口命令：
 *  h/? : help
 *  i   : print RF parameters
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

static void coord_print_help(void)
{
    dbg_puts("\r\n============= COORD(RF) HELP =============\r\n");
    dbg_puts("h/? : help\r\n");
    dbg_puts("i   : print RF parameters\r\n");
    dbg_puts("r   : force enter RX (continuous)\r\n");
    dbg_puts("=========================================\r\n");
}

static void coord_print_rf_info(void)
{
    dbg_puts("[COORD][RF] freq=");
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

static void coord_enter_rx_continuous(void)
{
    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_enter_continous_rx();
    dbg_puts("[COORD][RF] enter RX continuous\r\n");
}

static void coord_send_ack(uint8_t seq)
{
    uint8_t ack[16];
    uint8_t len = 0u;
    uint32_t t = (uint32_t)g_ms;

    ack[len++] = 'A';
    ack[len++] = 'C';
    ack[len++] = 'K';
    ack[len++] = seq;
    ack[len++] = (uint8_t)(t & 0xFFu);
    ack[len++] = (uint8_t)((t >> 8) & 0xFFu);
    ack[len++] = (uint8_t)((t >> 16) & 0xFFu);
    ack[len++] = (uint8_t)((t >> 24) & 0xFFu);

    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_send_packet(ack, (int)len);

    dbg_puts("[COORD][TX] ACK seq=0x");
    dbg_put_hex8(seq);
    dbg_puts(" len=");
    dbg_put_u32((uint32_t)len);
    dbg_puts("\r\n");
}

static void coord_handle_radio_events(void)
{
    int txf = rf_get_transmit_flag();
    int rxf = rf_get_recv_flag();

    if (txf == RADIO_FLAG_TXDONE)
    {
        (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
        dbg_puts("[COORD][IRQ] TX_DONE\r\n");
    }

    if (rxf == RADIO_FLAG_RXDONE)
    {
        uint8_t rx[128];
        uint8_t n;
        uint8_t seq = 0u;

        (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
        n = rf_recv_packet(rx);

        dbg_puts("[COORD][RX] len=");
        dbg_put_u32((uint32_t)n);
        dbg_puts(" data=");
        dbg_hexdump(rx, n);
        dbg_puts("\r\n");

        if (n >= 5u && rx[0] == 'P' && rx[1] == 'I' && rx[2] == 'N' && rx[3] == 'G')
        {
            seq = rx[4];
            dbg_puts("[COORD] got PING seq=0x");
            dbg_put_hex8(seq);
            dbg_puts(" -> reply ACK\r\n");

            coord_send_ack(seq);
        }

        /* 回到 RX */
        coord_enter_rx_continuous();
    }
    else if (rxf == RADIO_FLAG_RXTIMEOUT)
    {
        (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
        dbg_puts("[COORD][IRQ] RX_TIMEOUT\r\n");
        coord_enter_rx_continuous();
    }
    else if (rxf == RADIO_FLAG_RXERR)
    {
        (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
        dbg_puts("[COORD][IRQ] RX_CRC_ERR\r\n");
        coord_enter_rx_continuous();
    }
}

static RF_Err_t coord_rf_setup(void)
{
    RF_Err_t ret;

    dbg_puts("[COORD][RF] rf_init...\r\n");
    ret = rf_init();
    if (ret != RF_OK && ret != OK)
    {
        dbg_puts("[COORD][RF] rf_init FAIL\r\n");
        return RF_FAIL;
    }

    (void)rf_set_default_para();

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

    coord_print_rf_info();
    coord_enter_rx_continuous();
    return OK;
}

void app_coord_init(void)
{
    dbg_puts("\r\n[APP] ROLE=COORD (RAW RF)\r\n");
    coord_print_help();
    (void)coord_rf_setup();
}

void app_coord_task(void)
{
    coord_handle_radio_events();
}

void app_coord_on_cmd(uint8_t ch)
{
    if ((ch == (uint8_t)'h') || (ch == (uint8_t)'?'))
    {
        coord_print_help();
    }
    else if (ch == (uint8_t)'i')
    {
        coord_print_rf_info();
    }
    else if (ch == (uint8_t)'r')
    {
        coord_enter_rx_continuous();
    }
    else
    {
        dbg_puts("[COORD] unknown cmd: 0x");
        dbg_put_hex8(ch);
        dbg_puts("\r\n");
    }
}
