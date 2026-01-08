#include "dl2807_mac.h"
#include <string.h>
#include "pan3029_rf.h"

/* 强制引入你工程里的 CRC 实现（解决 implicit + CRC_TYPE_CCITT undefined） */
#include "crc.h"

/* 兜底：万一被其他同名 crc.h 抢走，也保证能编译 */
#ifndef CRC_TYPE_CCITT
#define CRC_TYPE_CCITT   (0u)
#endif

#ifndef CRC_TYPE_IBM
#define CRC_TYPE_IBM     (1u)
#endif

/* 兜底：防止 prototype 没进来导致 implicit */
#ifndef __RADIO_COMPUTE_CRC_PROTO__
#define __RADIO_COMPUTE_CRC_PROTO__
uint16_t RadioComputeCRC(uint8_t *buffer, uint8_t length, uint8_t crcType);
#endif

/* MAC格式：
 * [0] type
 * [1] seq
 * [2] pan L
 * [3] pan H
 * [4] src L
 * [5] src H
 * [6] dst L
 * [7] dst H
 * [8] payload_len
 * [9..] payload
 * [..] crc16 L
 * [..] crc16 H
 */
#define DL2807_MAC_HEADER_LEN   9
#define DL2807_MAC_CRC_LEN      2
#define DL2807_MAC_MIN_LEN      (DL2807_MAC_HEADER_LEN + DL2807_MAC_CRC_LEN)

static void dl2807_radio_basic_config(void);
static void dl2807_set_channel(dl2807_mac_ctx_t *ctx, dl2807_channel_t ch);
static void dl2807_radio_enter_rx(void);
static void dl2807_radio_send(dl2807_mac_ctx_t *ctx, uint8_t *buf, uint8_t len);

static uint8_t dl2807_build_frame(dl2807_mac_ctx_t *ctx,
                                  dl2807_frame_type_t type,
                                  uint16_t dst,
                                  const uint8_t *payload,
                                  uint8_t payload_len,
                                  uint8_t *out_buf);

static bool dl2807_parse_frame(const uint8_t *buf,
                               uint8_t len,
                               dl2807_mac_frame_t *out_frame);

static void dl2807_handle_rx_frame(dl2807_mac_ctx_t *ctx,
                                   const dl2807_mac_frame_t *frame);

static void dl2807_coordinator_superframe_fsm(dl2807_mac_ctx_t *ctx);
static void dl2807_node_superframe_fsm(dl2807_mac_ctx_t *ctx);

void dl2807_mac_init(dl2807_mac_ctx_t *ctx,
                     dl2807_role_t role,
                     uint16_t pan_id,
                     uint16_t node_id,
                     uint16_t coordinator_id)
{
    memset(ctx, 0, sizeof(dl2807_mac_ctx_t));

    ctx->role           = role;
    ctx->pan_id         = pan_id;
    ctx->node_id        = node_id;
    ctx->coordinator_id = coordinator_id;

    ctx->seq            = 0;
    ctx->current_ch     = DL2807_CH_CTRL;

    ctx->state          = (role == DL2807_ROLE_COORDINATOR)
                            ? DL2807_STATE_IDLE
                            : DL2807_STATE_WAIT_BEACON;

    ctx->synced         = (role == DL2807_ROLE_COORDINATOR);

    ctx->last_beacon_sf_id = 0xFFFFFFFFUL;

    rf_init();
	  rf_spi_self_test();
    dl2807_radio_basic_config();
    dl2807_set_channel(ctx, DL2807_CH_CTRL);
    dl2807_radio_enter_rx();
}

void dl2807_mac_1ms_tick(dl2807_mac_ctx_t *ctx)
{
    ctx->tick_ms++;
}

bool dl2807_mac_send_status(dl2807_mac_ctx_t *ctx,
                            const uint8_t *payload,
                            uint8_t len)
{
    if (len == 0u || len > DL2807_MAC_MAX_PAYLOAD_LEN)
        return false;

    if (ctx->app_tx_req)
        return false;

    memcpy(ctx->app_tx_payload, payload, len);
    ctx->app_tx_len = len;
    ctx->app_tx_req = true;

    return true;
}

/* ===== 控制信道 API：都是“排队”，由MAC在控制窗口发送 ===== */

static bool dl2807_ctrl_queue(dl2807_mac_ctx_t *ctx,
                              uint8_t ftype,
                              uint16_t dst,
                              const uint8_t *payload,
                              uint8_t len)
{
    if (len > DL2807_MAC_MAX_PAYLOAD_LEN)
        return false;

    if (ctx->ctrl_tx_req)
        return false; /* 只排队一帧，避免复杂化 */

    ctx->ctrl_tx_type = ftype;
    ctx->ctrl_tx_dst  = dst;
    ctx->ctrl_tx_len  = len;

    if (len > 0u && payload)
        memcpy(ctx->ctrl_tx_payload, payload, len);

    ctx->ctrl_tx_req = true;
    return true;
}

bool dl2807_mac_send_ctrl_req(dl2807_mac_ctx_t *ctx,
                              const uint8_t *payload,
                              uint8_t len)
{
    /* REQ默认发给 coordinator_id（如果还不知道 coordinator_id，就上层先保证 synced 后再发） */
    uint16_t dst = ctx->coordinator_id ? ctx->coordinator_id : 0xFFFF;
    return dl2807_ctrl_queue(ctx, (uint8_t)DL2807_FTYPE_CTRL_REQ, dst, payload, len);
}

bool dl2807_mac_send_ctrl_rsp(dl2807_mac_ctx_t *ctx,
                              uint16_t dst,
                              const uint8_t *payload,
                              uint8_t len)
{
    return dl2807_ctrl_queue(ctx, (uint8_t)DL2807_FTYPE_CTRL_RSP, dst, payload, len);
}

bool dl2807_mac_send_ctrl_ack(dl2807_mac_ctx_t *ctx,
                              uint16_t dst,
                              const uint8_t *payload,
                              uint8_t len)
{
    return dl2807_ctrl_queue(ctx, (uint8_t)DL2807_FTYPE_CTRL_ACK, dst, payload, len);
}

void dl2807_mac_task(dl2807_mac_ctx_t *ctx)
{
    /* 1) TX完成/超时 */
    if (ctx->tx_pending)
    {
        int tx_flag = rf_get_transmit_flag();

        if (tx_flag == RADIO_FLAG_TXDONE)
        {
            ctx->tx_pending = false;
            rf_set_transmit_flag(RADIO_FLAG_IDLE);
            dl2807_radio_enter_rx();
        }
        else if ((int32_t)(ctx->tick_ms - ctx->tx_deadline_ms) >= 0)
        {
            ctx->tx_pending = false;
            rf_set_transmit_flag(RADIO_FLAG_IDLE);
            dl2807_radio_enter_rx();
        }
    }

    /* 2) RX事件 */
    {
        int rx_flag = rf_get_recv_flag();

        if (rx_flag != RADIO_FLAG_IDLE)
        {
            if (rx_flag == RADIO_FLAG_RXDONE)
            {
                uint8_t len = rf_recv_packet(ctx->rx_buf);
                if (len > 0u && len <= (uint8_t)DL2807_MAC_FRAME_MAX_LEN)
                {
                    dl2807_mac_frame_t frame;
                    if (dl2807_parse_frame(ctx->rx_buf, len, &frame))
                    {
                        dl2807_handle_rx_frame(ctx, &frame);
                    }
                }
            }

            rf_set_recv_flag(RADIO_FLAG_IDLE);
            dl2807_radio_enter_rx();
        }
    }

    /* 3) 超帧时序 */
    if (ctx->role == DL2807_ROLE_COORDINATOR)
        dl2807_coordinator_superframe_fsm(ctx);
    else
        dl2807_node_superframe_fsm(ctx);

    /* 4) NODE 等RSP超时重试（最小实现） */
    if (ctx->role == DL2807_ROLE_NODE && ctx->ctrl_wait_rsp)
    {
        if ((int32_t)(ctx->tick_ms - ctx->ctrl_rsp_deadline_ms) >= 0)
        {
            ctx->ctrl_wait_rsp = false;

            if (ctx->ctrl_req_retry < DL2807_CTRL_REQ_RETRY_MAX)
            {
                ctx->ctrl_req_retry++;
                /* 重新排队REQ（保持payload不变） */
                /* 注意：如果此时队列已被占用则放弃本轮 */
                (void)dl2807_ctrl_queue(ctx,
                                        (uint8_t)DL2807_FTYPE_CTRL_REQ,
                                        ctx->coordinator_id ? ctx->coordinator_id : 0xFFFF,
                                        ctx->ctrl_tx_payload,
                                        ctx->ctrl_tx_len);
            }
        }
    }
}

/* ================= RF封装 ================= */

static void dl2807_radio_basic_config(void)
{
    rf_set_bw(DL2807_PHY_BW);
    rf_set_sf(DL2807_PHY_SF);
    rf_set_code_rate(DL2807_PHY_CR);
    rf_set_crc(CRC_ON);
    rf_set_preamble(DL2807_PHY_PREAMBLE);
    rf_set_syncword(DL2807_PHY_SYNCWORD);
    rf_set_dcdc_mode(DCDC_ON);
}

static void dl2807_set_channel(dl2807_mac_ctx_t *ctx, dl2807_channel_t ch)
{
    if (ctx->current_ch == ch)
        return;

    rf_set_mode(RF_MODE_STB3);

    if (ch == DL2807_CH_CTRL)
        rf_set_freq(DL2807_CTRL_FREQ_HZ);
    else
        rf_set_freq(DL2807_DATA_FREQ_HZ);

    ctx->current_ch = ch;
}

static void dl2807_radio_enter_rx(void)
{
    rf_set_mode(RF_MODE_STB3);
    rf_set_rx_mode(RF_RX_CONTINOUS);
    rf_set_mode(RF_MODE_RX);
}

static void dl2807_radio_send(dl2807_mac_ctx_t *ctx, uint8_t *buf, uint8_t len)
{
    rf_set_mode(RF_MODE_STB3);
    rf_set_tx_mode(RF_TX_SINGLE);

    rf_set_transmit_flag(RADIO_FLAG_IDLE);
    rf_send_packet(buf, len);

    ctx->tx_pending     = true;
    ctx->tx_len         = len;
    ctx->tx_deadline_ms = ctx->tick_ms + DL2807_TX_TIMEOUT_MS;
}

/* ================= MAC构造/解析 ================= */

static uint8_t dl2807_build_frame(dl2807_mac_ctx_t *ctx,
                                  dl2807_frame_type_t type,
                                  uint16_t dst,
                                  const uint8_t *payload,
                                  uint8_t payload_len,
                                  uint8_t *out_buf)
{
    uint8_t idx = 0;
    uint16_t crc16;

    if (payload_len > DL2807_MAC_MAX_PAYLOAD_LEN)
        payload_len = DL2807_MAC_MAX_PAYLOAD_LEN;

    out_buf[idx++] = (uint8_t)type;
    out_buf[idx++] = ctx->seq++;

    out_buf[idx++] = (uint8_t)(ctx->pan_id & 0xFF);
    out_buf[idx++] = (uint8_t)((ctx->pan_id >> 8) & 0xFF);

    out_buf[idx++] = (uint8_t)(ctx->node_id & 0xFF);
    out_buf[idx++] = (uint8_t)((ctx->node_id >> 8) & 0xFF);

    out_buf[idx++] = (uint8_t)(dst & 0xFF);
    out_buf[idx++] = (uint8_t)((dst >> 8) & 0xFF);

    out_buf[idx++] = payload_len;

    if (payload_len > 0u && payload)
    {
        memcpy(&out_buf[idx], payload, payload_len);
        idx = (uint8_t)(idx + payload_len);
    }

    crc16 = RadioComputeCRC(out_buf, (uint8_t)(DL2807_MAC_HEADER_LEN + payload_len), CRC_TYPE_CCITT);

    out_buf[idx++] = (uint8_t)(crc16 & 0xFF);
    out_buf[idx++] = (uint8_t)((crc16 >> 8) & 0xFF);

    return idx;
}

static bool dl2807_parse_frame(const uint8_t *buf,
                               uint8_t len,
                               dl2807_mac_frame_t *out_frame)
{
    uint8_t payload_len;
    uint16_t crc_calc;
    uint16_t crc_rx;

    if (len < (uint8_t)DL2807_MAC_MIN_LEN)
        return false;

    out_frame->type = buf[0];
    out_frame->seq  = buf[1];

    out_frame->pan_id = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    out_frame->src    = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    out_frame->dst    = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

    payload_len = buf[8];
    out_frame->payload_len = payload_len;

    if (payload_len > DL2807_MAC_MAX_PAYLOAD_LEN)
        return false;

    if (len != (uint8_t)(DL2807_MAC_HEADER_LEN + payload_len + DL2807_MAC_CRC_LEN))
        return false;

    if (payload_len > 0u)
        memcpy(out_frame->payload, &buf[9], payload_len);

    crc_rx = (uint16_t)buf[(uint8_t)(9 + payload_len)] |
             ((uint16_t)buf[(uint8_t)(10 + payload_len)] << 8);

		crc_calc = RadioComputeCRC(
				(uint8_t *)buf,   /* ARMCC5 下 CRC 接口不接受 const，这里安全 */
				(uint8_t)(DL2807_MAC_HEADER_LEN + payload_len),
				CRC_TYPE_CCITT );


    out_frame->crc16 = crc_rx; 

    return (crc_calc == crc_rx);
}

static void dl2807_handle_rx_frame(dl2807_mac_ctx_t *ctx,
                                   const dl2807_mac_frame_t *frame)
{
    /* 1) PAN过滤 */
    if (frame->pan_id != ctx->pan_id)
        return;

    /* 2) NODE: 收到BEACON -> 同步 */
    if (ctx->role == DL2807_ROLE_NODE)
    {
        if (frame->type == (uint8_t)DL2807_FTYPE_BEACON)
        {
            ctx->synced              = true;
            ctx->superframe_start_ms = ctx->tick_ms;
            ctx->coordinator_id      = frame->src;
        }

        /* NODE: 收到RSP -> 清等待并准备ACK（由上层决定是否回ACK） */
        if (frame->type == (uint8_t)DL2807_FTYPE_CTRL_RSP)
        {
            ctx->ctrl_wait_rsp = false;
            ctx->ctrl_req_retry = 0;
        }
    }

    /* 3) 目标地址过滤 */
    if (frame->dst != 0xFFFFu && frame->dst != ctx->node_id)
        return;

    /* 4) 回调上层 */
    dl2807_mac_on_frame_indication(ctx, frame);
}

/* ================= 超帧时序 ================= */

static void dl2807_coordinator_superframe_fsm(dl2807_mac_ctx_t *ctx)
{
    uint32_t t;
    uint32_t sf_id;

    if (!ctx->synced)
    {
        ctx->synced              = true;
        ctx->superframe_start_ms = ctx->tick_ms;
    }

    t = ctx->tick_ms - ctx->superframe_start_ms;
    if (t >= DL2807_SUPERFRAME_PERIOD_MS)
    {
        ctx->superframe_start_ms = ctx->tick_ms;
        t = 0;
    }

    sf_id = ctx->superframe_start_ms / DL2807_SUPERFRAME_PERIOD_MS;

    /* 控制窗口：先发BEACON（每超帧一次），再发控制应答（若有） */
    if (t < DL2807_CTRL_WINDOW_MS)
    {
        dl2807_set_channel(ctx, DL2807_CH_CTRL);

        /* 1) BEACON：每超帧仅发一次 */
        if (ctx->last_beacon_sf_id != sf_id)
        {
            if (!ctx->tx_pending)
            {
                uint8_t beacon_payload[4];

                ctx->state = DL2807_STATE_BEACON_TX;

                beacon_payload[0] = (uint8_t)(sf_id & 0xFF);
                beacon_payload[1] = (uint8_t)((sf_id >> 8) & 0xFF);
                beacon_payload[2] = (uint8_t)((sf_id >> 16) & 0xFF);
                beacon_payload[3] = (uint8_t)((sf_id >> 24) & 0xFF);

                {
                    uint8_t len = dl2807_build_frame(ctx,
                                                     DL2807_FTYPE_BEACON,
                                                     0xFFFFu,
                                                     beacon_payload,
                                                     (uint8_t)sizeof(beacon_payload),
                                                     ctx->tx_buf);
                    dl2807_radio_send(ctx, ctx->tx_buf, len);
                    ctx->last_beacon_sf_id = sf_id;
                }
            }
            return;
        }

        /* 2) 控制应答帧：若队列里有RSP/ACK等，且当前不在TX */
        if (ctx->ctrl_tx_req && !ctx->tx_pending)
        {
            uint8_t len = dl2807_build_frame(ctx,
                                             (dl2807_frame_type_t)ctx->ctrl_tx_type,
                                             ctx->ctrl_tx_dst,
                                             ctx->ctrl_tx_payload,
                                             ctx->ctrl_tx_len,
                                             ctx->tx_buf);
            dl2807_radio_send(ctx, ctx->tx_buf, len);
            ctx->ctrl_tx_req = false;
        }
        return;
    }

    /* 业务窗口：DATA_DOWN（原样保留） */
    if (t < (DL2807_CTRL_WINDOW_MS + DL2807_DATA_UP_WINDOW_MS))
    {
        ctx->state = DL2807_STATE_DATA_TX;
        dl2807_set_channel(ctx, DL2807_CH_DATA);

        if (ctx->app_tx_req && !ctx->tx_pending)
        {
            uint16_t dst = 0xFFFFu;
            uint8_t len = dl2807_build_frame(ctx,
                                             DL2807_FTYPE_DATA_DOWN,
                                             dst,
                                             ctx->app_tx_payload,
                                             ctx->app_tx_len,
                                             ctx->tx_buf);
            dl2807_radio_send(ctx, ctx->tx_buf, len);
            ctx->app_tx_req = false;
        }
        return;
    }

    /* 其它时间：RX */
    ctx->state = DL2807_STATE_DATA_RX;
    dl2807_set_channel(ctx, DL2807_CH_DATA);
}

static void dl2807_node_superframe_fsm(dl2807_mac_ctx_t *ctx)
{
    uint32_t t;

    if (!ctx->synced)
    {
        ctx->state = DL2807_STATE_WAIT_BEACON;
        dl2807_set_channel(ctx, DL2807_CH_CTRL);
        return;
    }

    t = ctx->tick_ms - ctx->superframe_start_ms;

    if (t >= DL2807_SUPERFRAME_PERIOD_MS)
    {
        ctx->synced = false;
        return;
    }

    /* 控制窗口：监听/发送控制REQ/ACK */
    if (t < DL2807_CTRL_WINDOW_MS)
    {
        ctx->state = DL2807_STATE_WAIT_BEACON;
        dl2807_set_channel(ctx, DL2807_CH_CTRL);

        if (ctx->ctrl_tx_req && !ctx->tx_pending)
        {
            uint8_t len = dl2807_build_frame(ctx,
                                             (dl2807_frame_type_t)ctx->ctrl_tx_type,
                                             ctx->ctrl_tx_dst,
                                             ctx->ctrl_tx_payload,
                                             ctx->ctrl_tx_len,
                                             ctx->tx_buf);

            dl2807_radio_send(ctx, ctx->tx_buf, len);
            ctx->ctrl_tx_req = false;

            /* 如果刚发的是REQ，则开始等待RSP */
            if (ctx->ctrl_tx_type == (uint8_t)DL2807_FTYPE_CTRL_REQ)
            {
                ctx->ctrl_wait_rsp = true;
                ctx->ctrl_rsp_deadline_ms = ctx->tick_ms + DL2807_CTRL_WAIT_RSP_MS;
            }
        }
        return;
    }

    /* 业务窗口：DATA_UP（原样保留） */
    if (t < (DL2807_CTRL_WINDOW_MS + DL2807_DATA_UP_WINDOW_MS))
    {
        ctx->state = DL2807_STATE_DATA_TX;
        dl2807_set_channel(ctx, DL2807_CH_DATA);

        if (ctx->app_tx_req && !ctx->tx_pending)
        {
            uint8_t len = dl2807_build_frame(ctx,
                                             DL2807_FTYPE_DATA_UP,
                                             ctx->coordinator_id,
                                             ctx->app_tx_payload,
                                             ctx->app_tx_len,
                                             ctx->tx_buf);

            dl2807_radio_send(ctx, ctx->tx_buf, len);
            ctx->app_tx_req = false;
        }
        return;
    }

    ctx->state = DL2807_STATE_DATA_RX;
    dl2807_set_channel(ctx, DL2807_CH_DATA);
}
