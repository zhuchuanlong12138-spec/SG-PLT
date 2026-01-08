/******************************************************************************
 * APP_COORD_main.c - Coordinator application layer
 *
 * Implements:
 *   app_coord_init()
 *   app_coord_task()
 *   app_coord_on_cmd()
 *   app_coord_on_frame()
 *
 * IMPORTANT:
 *   Delay_* is implemented in delay.c/delay.h. Do NOT duplicate it here,
 *   otherwise the linker will report L6200E: multiply defined.
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dbg.h"
#include "dl2807_mac.h"

/* Provided by main.c */
extern dl2807_mac_ctx_t g_mac;
extern volatile uint32_t g_ms;

#define COORD_RSP_MAX_LEN   (32u)

/* Simple demo payload format:
 *  REQ: [0]=0xA1, [1]=echo
 *  RSP: [0]=0xB1, [1]=echo, [2..5]=t_ms (LE)
 *  ACK: [0]=0xC1, [1]=echo
 */

static void coord_print_help(void)
{
    dbg_puts("\r\n============= COORD HELP =============\r\n");
    dbg_puts("h/? : help\r\n");
    dbg_puts("i   : print coordinator info\r\n");
    dbg_puts("======================================\r\n");
}

static void coord_print_info(void)
{
    dbg_puts("[COORD] pan=0x");
    dbg_put_hex16(g_mac.pan_id);

    dbg_puts(" id=0x");
    dbg_put_hex16(g_mac.node_id);

    dbg_puts(" synced=");
    dbg_put_u32((uint32_t)(g_mac.synced ? 1u : 0u));

    dbg_puts(" ch=");
    dbg_put_u32((uint32_t)g_mac.current_ch);

    dbg_puts(" state=");
    dbg_put_u32((uint32_t)g_mac.state);

    dbg_puts("\r\n");
}

void app_coord_init(void)
{
    dbg_puts("\r\n[APP] ROLE=COORD\r\n");
    coord_print_info();
    coord_print_help();
}

void app_coord_task(void)
{
    /* nothing periodic for minimal demo */
}

void app_coord_on_cmd(uint8_t ch)
{
    if ((ch == (uint8_t)'h') || (ch == (uint8_t)'?'))
    {
        coord_print_help();
    }
    else if (ch == (uint8_t)'i')
    {
        coord_print_info();
    }
    else
    {
        dbg_puts("[COORD] unknown cmd: 0x");
        dbg_put_hex8(ch);
        dbg_puts("\r\n");
    }
}

void app_coord_on_frame(const dl2807_mac_frame_t *frame)
{
    uint8_t rsp[COORD_RSP_MAX_LEN];
    uint8_t rsp_len;
    uint8_t echo;

    if (frame == (const dl2807_mac_frame_t *)0)
    {
        return;
    }

    if (frame->type == (uint8_t)DL2807_FTYPE_CTRL_REQ)
    {
        echo = 0u;
        if (frame->payload_len >= 2u)
        {
            echo = frame->payload[1];
        }

        dbg_puts("[COORD] CTRL_REQ from 0x");
        dbg_put_hex16(frame->src);
        dbg_puts(" echo=0x");
        dbg_put_hex8(echo);
        dbg_puts("\r\n");

        rsp_len = 0u;
        rsp[rsp_len++] = 0xB1u;
        rsp[rsp_len++] = echo;
        rsp[rsp_len++] = (uint8_t)(g_ms & 0xFFu);
        rsp[rsp_len++] = (uint8_t)((g_ms >> 8) & 0xFFu);
        rsp[rsp_len++] = (uint8_t)((g_ms >> 16) & 0xFFu);
        rsp[rsp_len++] = (uint8_t)((g_ms >> 24) & 0xFFu);

        (void)dl2807_mac_send_ctrl_rsp(&g_mac, frame->src, rsp, rsp_len);
    }
    else if (frame->type == (uint8_t)DL2807_FTYPE_CTRL_ACK)
    {
        dbg_puts("[COORD] CTRL_ACK from 0x");
        dbg_put_hex16(frame->src);

        if (frame->payload_len > 0u)
        {
            dbg_puts(" payload0=0x");
            dbg_put_hex8(frame->payload[0]);
        }
        dbg_puts("\r\n");
    }
    else
    {
        /* other frames printed in main.c already */
    }
}
