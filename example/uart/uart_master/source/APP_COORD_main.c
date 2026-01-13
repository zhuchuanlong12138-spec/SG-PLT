/******************************************************************************
 * APP_COORD_main.c - PAN3029 Raw RF Coordinator (TX broadcaster)
 *
 * 需求实现：
 *  - COORD 每 1 秒发送一个自增的数（uint32_t counter++）
 *  - NODE 常开 RX，收到后打印出来
 *
 * 帧格式（小端）：
 *   'C' 'N' 'T' counter32
 *
 * 串口命令：
 *  h/? : help
 *  i   : print RF parameters
 *  t   : send once now
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

#define COORD_TX_PERIOD_MS    (1000u)
#define COORD_PKT_MAX_LEN     (16u)

/* ============= 内部状态 ============= */
static uint32_t s_next_tx_ms = 0u;
static uint32_t s_counter    = 0u;
static uint8_t  s_rf_ready   = 0u;  /* rf_init + 参数配置完成标志 */

/* ============= 小工具：无符号比较（处理 g_ms 溢出） ============= */
static bool time_reached(uint32_t now, uint32_t target)
{
    /* (int32_t)(now - target) >= 0 等价于 now >= target 且可处理溢出 */
    return ((int32_t)(now - target) >= 0) ? true : false;
}

static void coord_print_help(void)
{
    dbg_puts("\r\n============= COORD(RF) HELP =============\r\n");
    dbg_puts("h/? : help\r\n");
    dbg_puts("i   : print RF parameters\r\n");
    dbg_puts("t   : send once now\r\n");
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

/* 发送计数包：'C''N''T'+u32le */
static void coord_send_counter(uint32_t v)
{
    uint8_t pkt[COORD_PKT_MAX_LEN];
    uint8_t len = 0u;
    int txf;

    if (s_rf_ready == 0u)
    {
        dbg_puts("[COORD][TX] RF not ready, ignore\r\n");
        return;
    }

    /* 如果上一次 TX 还没完成，不要继续发，避免把状态打乱 */
    txf = rf_get_transmit_flag();
    if (txf != RADIO_FLAG_IDLE && txf != RADIO_FLAG_TXDONE)
    {
        dbg_puts("[COORD][TX] busy, tx_flag=");
        dbg_put_u32((uint32_t)txf);
        dbg_puts("\r\n");
        return;
    }

    pkt[len++] = 'C';
    pkt[len++] = 'N';
    pkt[len++] = 'T';
    pkt[len++] = (uint8_t)(v & 0xFFu);
    pkt[len++] = (uint8_t)((v >> 8) & 0xFFu);
    pkt[len++] = (uint8_t)((v >> 16) & 0xFFu);
    pkt[len++] = (uint8_t)((v >> 24) & 0xFFu);

    /* 清一次 IRQ，避免历史中断影响状态机 */
    (void)rf_clr_irq(0xFFu);

    /* 强制置为 IDLE，再发 */
    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);

    /* 发送 */
    (void)rf_send_packet(pkt, (int)len);

    dbg_puts("[COORD][TX] CNT=");
    dbg_put_u32(v);
    dbg_puts(" len=");
    dbg_put_u32((uint32_t)len);
    dbg_puts("\r\n");
}

/* 处理 TX 完成事件（如果你后面要做统计，这里是入口） */
static void coord_handle_radio_events(void)
{
    int txf = rf_get_transmit_flag();

    if (txf == RADIO_FLAG_TXDONE)
    {
        (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
        dbg_puts("[COORD][IRQ] TX_DONE\r\n");
    }
}

/* RF 配置：rf_init + 设置一致参数 */
static RF_Err_t coord_rf_setup(void)
{
    RF_Err_t ret;

    dbg_puts("[COORD][RF] rf_init...\r\n");
    ret = rf_init();
    if (ret != OK && ret != RF_OK)
    {
        dbg_puts("[COORD][RF] rf_init FAIL\r\n");
        s_rf_ready = 0u;
        return RF_FAIL;
    }

    /* 默认参数（库函数内部会写很多寄存器） */
  //  (void)rf_set_default_para();

    /* 关键参数（必须两端一致） */
    (void)rf_set_freq(APP_RF_FREQ_HZ);
    (void)rf_set_bw(APP_RF_BW);
    (void)rf_set_sf(APP_RF_SF);
    (void)rf_set_code_rate(APP_RF_CR);
    (void)rf_set_syncword(APP_RF_SYNC);
    (void)rf_set_crc((APP_RF_CRC_ON == CRC_ON) ? true : false);
    (void)rf_set_tx_power(APP_RF_TXPWR);

    /* 旗标/中断清理 */
    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
    (void)rf_clr_irq(0xFFu);

    coord_print_rf_info();

    /* 第一次延后 1 秒发送 */
    s_next_tx_ms = (uint32_t)g_ms + COORD_TX_PERIOD_MS;
    s_counter = 0u;
    s_rf_ready = 1u;

    dbg_puts("[COORD][RF] setup OK\r\n");
    return OK;
}

//static void coord_dbg_step(const char *tag)
//{
//    dbg_puts(tag);
//    dbg_puts(" ms=");
//    dbg_put_u32((uint32_t)g_ms);
//    dbg_puts(" r04=0x");
//    dbg_put_hex8(rf_read_reg(0x04u));
//    dbg_puts(" sys=0x");
//    dbg_put_hex8(rf_read_reg(REG_SYS_CTL));
//    dbg_puts(" mode=");
//    dbg_put_u32((uint32_t)rf_get_mode());
//    dbg_puts("\r\n");
//}

//static RF_Err_t coord_rf_setup(void)
//{
//    RF_Err_t ret;

//    dbg_puts("[COORD][RF] rf_init...\r\n");
//    ret = rf_init();
//    dbg_puts("[COORD][RF] rf_init ret=");
//    dbg_put_u32((uint32_t)ret);
//    dbg_puts("\r\n");

//    if (ret != OK && ret != RF_OK)
//    {
//        dbg_puts("[COORD][RF] rf_init FAIL\r\n");
//        s_rf_ready = 0u;
//        return RF_FAIL;
//    }

//    coord_dbg_step("[COORD][RF] STEP0 after rf_init");

//    /* ---------------- STEP1: default para ---------------- */
////    dbg_puts("[COORD][RF] STEP1 rf_set_default_para begin\r\n");
////    (void)rf_set_default_para();
////    dbg_puts("[COORD][RF] STEP1 rf_set_default_para end\r\n");
////    coord_dbg_step("[COORD][RF] STEP1 after default_para");

//    /* ---------------- STEP2: set freq ---------------- */
//    dbg_puts("[COORD][RF] STEP2 rf_set_freq begin\r\n");
//    (void)rf_set_freq(APP_RF_FREQ_HZ);
//    dbg_puts("[COORD][RF] STEP2 rf_set_freq end\r\n");
//    dbg_puts("[COORD][RF] freq_read=");
//    dbg_put_u32((uint32_t)rf_read_freq());
//    dbg_puts("\r\n");
//    coord_dbg_step("[COORD][RF] STEP2 after set_freq");

//    /* ---------------- STEP3: set bw ---------------- */
//    dbg_puts("[COORD][RF] STEP3 rf_set_bw begin\r\n");
//    (void)rf_set_bw(APP_RF_BW);
//    dbg_puts("[COORD][RF] STEP3 rf_set_bw end\r\n");
//    dbg_puts("[COORD][RF] bw_get=");
//    dbg_put_u32((uint32_t)rf_get_bw());
//    dbg_puts("\r\n");
//    coord_dbg_step("[COORD][RF] STEP3 after set_bw");

//    /* ---------------- STEP4: set sf ---------------- */
//    dbg_puts("[COORD][RF] STEP4 rf_set_sf begin\r\n");
//    (void)rf_set_sf(APP_RF_SF);
//    dbg_puts("[COORD][RF] STEP4 rf_set_sf end\r\n");
//    dbg_puts("[COORD][RF] sf_get=");
//    dbg_put_u32((uint32_t)rf_get_sf());
//    dbg_puts("\r\n");
//    coord_dbg_step("[COORD][RF] STEP4 after set_sf");

//    /* ---------------- STEP5: set cr ---------------- */
//    dbg_puts("[COORD][RF] STEP5 rf_set_code_rate begin\r\n");
//    (void)rf_set_code_rate(APP_RF_CR);
//    dbg_puts("[COORD][RF] STEP5 rf_set_code_rate end\r\n");
//    dbg_puts("[COORD][RF] cr_get=");
//    dbg_put_u32((uint32_t)rf_get_code_rate());
//    dbg_puts("\r\n");
//    coord_dbg_step("[COORD][RF] STEP5 after set_cr");

//    /* ---------------- STEP6: syncword ---------------- */
//    dbg_puts("[COORD][RF] STEP6 rf_set_syncword begin\r\n");
//    (void)rf_set_syncword(APP_RF_SYNC);
//    dbg_puts("[COORD][RF] STEP6 rf_set_syncword end\r\n");
//    dbg_puts("[COORD][RF] sync_get=0x");
//    dbg_put_hex8(rf_get_syncword());
//    dbg_puts("\r\n");
//    coord_dbg_step("[COORD][RF] STEP6 after set_sync");

//    /* ---------------- STEP7: crc ---------------- */
//    dbg_puts("[COORD][RF] STEP7 rf_set_crc begin\r\n");
//    (void)rf_set_crc((APP_RF_CRC_ON == CRC_ON) ? true : false);
//    dbg_puts("[COORD][RF] STEP7 rf_set_crc end\r\n");
//    dbg_puts("[COORD][RF] crc_get=");
//    dbg_put_u32((uint32_t)rf_get_crc());
//    dbg_puts("\r\n");
//    coord_dbg_step("[COORD][RF] STEP7 after set_crc");

//    /* ---------------- STEP8: tx power ---------------- */
//    dbg_puts("[COORD][RF] STEP8 rf_set_tx_power begin\r\n");
//    (void)rf_set_tx_power(APP_RF_TXPWR);
//    dbg_puts("[COORD][RF] STEP8 rf_set_tx_power end\r\n");
//    dbg_puts("[COORD][RF] txpwr_get=");
//    dbg_put_u32((uint32_t)rf_get_tx_power());
//    dbg_puts("\r\n");
//    coord_dbg_step("[COORD][RF] STEP8 after set_txpwr");

//    /* ---------------- STEP9: flags/irq ---------------- */
//    dbg_puts("[COORD][RF] STEP9 flags/irq begin\r\n");
//    (void)rf_set_recv_flag(RADIO_FLAG_IDLE);
//    (void)rf_set_transmit_flag(RADIO_FLAG_IDLE);
//    (void)rf_clr_irq(0xFFu);
//    dbg_puts("[COORD][RF] STEP9 flags/irq end\r\n");
//    coord_dbg_step("[COORD][RF] STEP9 after clr_irq");

//    /* 这里：你原来卡在“看不到 print_rf_info”，现在我们先打印一个“到达点” */
//    dbg_puts("[COORD][RF] STEP10 about to print_rf_info\r\n");
//    coord_print_rf_info();
//    dbg_puts("[COORD][RF] STEP10 print_rf_info done\r\n");

//    /* 第一次延后 1 秒发送 */
//    s_next_tx_ms = (uint32_t)g_ms + COORD_TX_PERIOD_MS;
//    s_counter = 0u;
//    s_rf_ready = 1u;

//    dbg_puts("[COORD][RF] setup OK\r\n");
//    return OK;
//}


void app_coord_init(void)
{
    dbg_puts("\r\n[APP] ROLE=COORD (RAW RF, TX broadcaster)\r\n");
    coord_print_help();
    (void)coord_rf_setup();
}

/* 主循环里需要被周期调用 */
void app_coord_task(void)
{
    uint32_t now = (uint32_t)g_ms;

    coord_handle_radio_events();

    if (s_rf_ready == 0u)
    {
        return;
    }

    if (time_reached(now, s_next_tx_ms))
    {
        s_counter++;
        coord_send_counter(s_counter);
        s_next_tx_ms = now + COORD_TX_PERIOD_MS;
    }
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
    else if (ch == (uint8_t)'t')
    {
        /* 手动触发一次发送（方便你验证：即使 main 没调 task，也能发） */
        s_counter++;
        coord_send_counter(s_counter);
        s_next_tx_ms = (uint32_t)g_ms + COORD_TX_PERIOD_MS;
    }
    else
    {
        dbg_puts("[COORD] unknown cmd: 0x");
        dbg_put_hex8(ch);
        dbg_puts("\r\n");
    }
}
