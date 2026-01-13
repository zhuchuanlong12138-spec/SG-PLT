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
extern struct RxDoneMsg RxDoneParams;

/* =================== RF 参数（两块板必须一致） =================== */
#ifndef APP_RF_FREQ_HZ
#define APP_RF_FREQ_HZ        (433000000UL)
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

/* 在不引入 printf/浮点的前提下打印有符号 8 位数 */
static void node_put_i8(int8_t v)
{
    if (v < 0)
    {
        dbg_puts("-");
        dbg_put_u32((uint32_t)(-v));
    }
    else
    {
        dbg_put_u32((uint32_t)v);
    }
}

static void node_handle_radio_events(void)
{
    int rxf = rf_get_recv_flag();

    if (rxf == RADIO_FLAG_RXDONE)
    {
        (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
        /*
         * 注意：rf_irq_process() 在检测到 REG_IRQ_RX_DONE 后，
         * 已经把 FIFO 的 payload 读到全局 RxDoneParams.Payload，
         * 并把长度/RS SI/SNR 存到 RxDoneParams 里。
         * 这里不要再调用 rf_recv_packet()，否则会把 FIFO 读空/读错。
         */

        dbg_puts("[NODE][RX] len=");
        dbg_put_u32((uint32_t)RxDoneParams.Size);
        dbg_puts(" rssi=");
        node_put_i8((int8_t)RxDoneParams.Rssi);
        dbg_puts(" data=");
        dbg_hexdump(RxDoneParams.Payload, (uint32_t)RxDoneParams.Size);
        dbg_puts("\r\n");

        /* 解析 CNT 帧 */
        if (RxDoneParams.Size >= 7u && RxDoneParams.Payload[0] == 'C' && RxDoneParams.Payload[1] == 'N' && RxDoneParams.Payload[2] == 'T')
        {
            uint32_t v = 0u;
            v |= (uint32_t)RxDoneParams.Payload[3];
            v |= ((uint32_t)RxDoneParams.Payload[4] << 8);
            v |= ((uint32_t)RxDoneParams.Payload[5] << 16);
            v |= ((uint32_t)RxDoneParams.Payload[6] << 24);

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

    /*
     * 默认参数（强烈建议两端都调用）
     * 说明：库内部会写入一组推荐寄存器，避免出现“隐藏配置不一致”。
     */
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
