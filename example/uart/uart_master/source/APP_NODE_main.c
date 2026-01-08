/******************************************************************************
 * APP_NODE_main.c - Node application layer
 *
 * Minimal demo behavior:
 *  - Periodically send CTRL_REQ to coordinator
 *  - When CTRL_RSP received -> send CTRL_ACK
 *
 * NOTE:
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

#define NODE_REQ_PERIOD_MS   (3000u)

/* Simple demo payload format:
 *  REQ: [0]=0xA1, [1]=echo(counter)
 *  RSP: [0]=0xB1, [1]=echo(counter), [2..5]=t_ms (LE)
 *  ACK: [0]=0xC1, [1]=echo(counter)
 */

static uint32_t s_next_req_ms = 0u;
static uint8_t  s_echo = 0u;

static void node_print_help(void)
{
    dbg_puts("\r\n============= NODE HELP =============\r\n");
    dbg_puts("h/? : help\r\n");
    dbg_puts("i   : print node info\r\n");
    dbg_puts("r   : send one CTRL_REQ now\r\n");
    dbg_puts("====================================\r\n");
}

static void node_print_info(void)
{
    dbg_puts("[NODE] pan=0x");
    dbg_put_hex16(g_mac.pan_id);

    dbg_puts(" id=0x");
    dbg_put_hex16(g_mac.node_id);

    dbg_puts(" coord=0x");
    dbg_put_hex16(g_mac.coordinator_id);

    dbg_puts(" synced=");
    dbg_put_u32((uint32_t)(g_mac.synced ? 1u : 0u));

    dbg_puts(" ch=");
    dbg_put_u32((uint32_t)g_mac.current_ch);

    dbg_puts(" state=");
    dbg_put_u32((uint32_t)g_mac.state);

    dbg_puts("\r\n");
}

static void node_send_req(void)
{
    uint8_t payload[2];

    s_echo++;
    payload[0] = 0xA1u;
    payload[1] = s_echo;

    dbg_puts("[NODE] send CTRL_REQ echo=0x");
    dbg_put_hex8(s_echo);
    dbg_puts("\r\n");

    (void)dl2807_mac_send_ctrl_req(&g_mac, payload, 2u);
}

void app_node_init(void)
{
    dbg_puts("\r\n[APP] ROLE=NODE\r\n");
    node_print_info();
    node_print_help();

    s_next_req_ms = (uint32_t)g_ms + 1000u;
}

void app_node_task(void)
{
    if ((uint32_t)g_ms >= s_next_req_ms)
    {
        node_send_req();
        s_next_req_ms = (uint32_t)g_ms + NODE_REQ_PERIOD_MS;
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
        node_print_info();
    }
    else if (ch == (uint8_t)'r')
    {
        node_send_req();
        s_next_req_ms = (uint32_t)g_ms + NODE_REQ_PERIOD_MS;
    }
    else
    {
        dbg_puts("[NODE] unknown cmd: 0x");
        dbg_put_hex8(ch);
        dbg_puts("\r\n");
    }
}

void app_node_on_frame(const dl2807_mac_frame_t *frame)
{
    uint8_t ack[2];
    uint8_t echo;

    if (frame == (const dl2807_mac_frame_t *)0)
    {
        return;
    }

    if (frame->type == (uint8_t)DL2807_FTYPE_CTRL_RSP)
    {
        echo = 0u;
        if (frame->payload_len >= 2u)
        {
            echo = frame->payload[1];
        }

        dbg_puts("[NODE] CTRL_RSP from 0x");
        dbg_put_hex16(frame->src);
        dbg_puts(" echo=0x");
        dbg_put_hex8(echo);
        dbg_puts(" -> send ACK\r\n");

        ack[0] = 0xC1u;
        ack[1] = echo;
        (void)dl2807_mac_send_ctrl_ack(&g_mac, frame->src, ack, 2u);
    }
    else
    {
        /* other frames printed in main.c already */
    }
}
