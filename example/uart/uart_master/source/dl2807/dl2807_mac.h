#ifndef DL2807_MAC_H
#define DL2807_MAC_H

#include <stdint.h>
#include <stdbool.h>
#include "radio.h"
#include "crc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DL2807_MAC_MAX_PAYLOAD_LEN     32
#define DL2807_MAC_FRAME_MAX_LEN       (1 + 1 + 2 + 2 + 2 + 1 + DL2807_MAC_MAX_PAYLOAD_LEN + 2)

#define DL2807_MAC_DEFAULT_PAN_ID      0x2807

#define DL2807_CTRL_FREQ_HZ            490300000UL
#define DL2807_DATA_FREQ_HZ            490700000UL

#define DL2807_PHY_BW                  BW_125K
#define DL2807_PHY_SF                  SF_9
#define DL2807_PHY_CR                  CODE_RATE_47
#define DL2807_PHY_PREAMBLE            8
#define DL2807_PHY_SYNCWORD            0x12

#define DL2807_SUPERFRAME_PERIOD_MS    1000U
#define DL2807_CTRL_WINDOW_MS          80U
#define DL2807_DATA_UP_WINDOW_MS       200U
#define DL2807_GUARD_WINDOW_MS         20U

#define DL2807_TX_TIMEOUT_MS           300U

/* ŵȴ/ԲСʵ֣ڴӡ򵥳ʱж */
#define DL2807_CTRL_WAIT_RSP_MS        110U
#define DL2807_CTRL_REQ_RETRY_MAX      2U

typedef enum
{
    DL2807_ROLE_COORDINATOR = 0,
    DL2807_ROLE_NODE        = 1,
} dl2807_role_t;

typedef enum
{
    DL2807_CH_CTRL = 0,
    DL2807_CH_DATA = 1,
} dl2807_channel_t;

typedef enum
{
    DL2807_FTYPE_BEACON       = 0x01,
    DL2807_FTYPE_JOIN_REQ     = 0x02,
    DL2807_FTYPE_JOIN_ACK     = 0x03,

    DL2807_FTYPE_DATA_UP      = 0x10,
    DL2807_FTYPE_DATA_DOWN    = 0x11,

    /* ===== ŵСջREQ/RSP/ACK ===== */
    DL2807_FTYPE_CTRL_REQ     = 0x30,
    DL2807_FTYPE_CTRL_RSP     = 0x31,
    DL2807_FTYPE_CTRL_ACK     = 0x32,

    /*  */
    DL2807_FTYPE_ACK          = 0x20,
} dl2807_frame_type_t;

typedef struct
{
    uint8_t  type;
    uint8_t  seq;
    uint16_t pan_id;
    uint16_t src;
    uint16_t dst;
    uint8_t  payload_len;
    uint8_t  payload[DL2807_MAC_MAX_PAYLOAD_LEN];
    uint16_t crc16;
} dl2807_mac_frame_t;

typedef enum
{
    DL2807_STATE_IDLE = 0,
    DL2807_STATE_WAIT_BEACON,
    DL2807_STATE_BEACON_TX,
    DL2807_STATE_DATA_TX,
    DL2807_STATE_DATA_RX,
} dl2807_mac_state_t;

typedef struct
{
    dl2807_role_t      role;
    dl2807_mac_state_t state;

    uint16_t pan_id;
    uint16_t node_id;
    uint16_t coordinator_id;

    uint8_t  seq;
    bool     synced;
    uint32_t tick_ms;
    uint32_t superframe_start_ms;

    dl2807_channel_t current_ch;

    uint8_t  tx_buf[DL2807_MAC_FRAME_MAX_LEN];
    uint8_t  rx_buf[DL2807_MAC_FRAME_MAX_LEN];

    bool     tx_pending;
    uint8_t  tx_len;
    uint32_t tx_deadline_ms;

    bool     rx_pending;
    uint8_t  rx_len;

    /* ҵDATA_UP/DOWN */
    bool     app_tx_req;
    uint8_t  app_tx_len;
    uint8_t  app_tx_payload[DL2807_MAC_MAX_PAYLOAD_LEN];

    /* ===== ŵУREQ/RSP/ACK===== */
    bool     ctrl_tx_req;
    uint8_t  ctrl_tx_len;
    uint8_t  ctrl_tx_payload[DL2807_MAC_MAX_PAYLOAD_LEN];
    uint16_t ctrl_tx_dst;
    uint8_t  ctrl_tx_type; /* DL2807_FTYPE_CTRL_REQ/RSP/ACK */

    /* NODE: ȴRSP */
    bool     ctrl_wait_rsp;
    uint32_t ctrl_rsp_deadline_ms;
    uint8_t  ctrl_req_retry;

    /* COORD: beaconȥ */
    uint32_t last_beacon_sf_id;
} dl2807_mac_ctx_t;

void dl2807_mac_init(dl2807_mac_ctx_t *ctx,
                     dl2807_role_t role,
                     uint16_t pan_id,
                     uint16_t node_id,
                     uint16_t coordinator_id);

void dl2807_mac_1ms_tick(dl2807_mac_ctx_t *ctx);

void dl2807_mac_task(dl2807_mac_ctx_t *ctx);

/* ҵϱԭУ */
bool dl2807_mac_send_status(dl2807_mac_ctx_t *ctx,
                            const uint8_t *payload,
                            uint8_t len);

/* ===== ŵСջ API ===== */

/* NODEREQĿĬ coordinator_idδ֪ɴ dst=0xFFFF ϲԼ */
bool dl2807_mac_send_ctrl_req(dl2807_mac_ctx_t *ctx,
                              const uint8_t *payload,
                              uint8_t len);

/* COORDRSP ĳڵ */
bool dl2807_mac_send_ctrl_rsp(dl2807_mac_ctx_t *ctx,
                              uint16_t dst,
                              const uint8_t *payload,
                              uint8_t len);

/* NODEACK  COORDһյRSPã */
bool dl2807_mac_send_ctrl_ack(dl2807_mac_ctx_t *ctx,
                              uint16_t dst,
                              const uint8_t *payload,
                              uint8_t len);

void dl2807_mac_on_frame_indication(const dl2807_mac_ctx_t *ctx,
                                    const dl2807_mac_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
