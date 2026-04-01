#include "can_usb_bridge.h"
#include "main.h"
#include "led.h"
#include <string.h>

#define BSU_PREAMBLE_LO    0x55u
#define BSU_PREAMBLE_HI    0xAAu
#define BSU_HEADER_SIZE    (2u + 2u + 2u + 2u)
#define BSU_CAN_PAYLOAD    (4u + 8u)
#define BSU_CHECKSUM_SIZE  2u
#define BSU_CAN_PKT_SIZE   (BSU_HEADER_SIZE + BSU_CAN_PAYLOAD + BSU_CHECKSUM_SIZE)

#define BRIDGE_QUEUE_SIZE  128u

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t bus; /* 1=CAN1, 2=CAN2 */
} BridgeCanFrame;

typedef enum {
    RX_PREAMBLE_0 = 0,
    RX_PREAMBLE_1,
    RX_SIZE_LO,
    RX_SIZE_HI,
    RX_TYPE_LO,
    RX_TYPE_HI,
    RX_SEQ_LO,
    RX_SEQ_HI,
    RX_BODY,
    RX_CRC_LO,
    RX_CRC_HI
} RxState;

static BridgeCanFrame g_can_to_usb_q[BRIDGE_QUEUE_SIZE];
static volatile uint16_t g_can_to_usb_head = 0u;
static volatile uint16_t g_can_to_usb_tail = 0u;

static BridgeCanFrame g_usb_to_can_q[BRIDGE_QUEUE_SIZE];
static volatile uint16_t g_usb_to_can_head = 0u;
static volatile uint16_t g_usb_to_can_tail = 0u;

static uint8_t g_usb_tx_pkt[BSU_CAN_PKT_SIZE];
static volatile uint8_t g_usb_tx_busy = 0u;
static uint32_t g_usb_tx_start_tick = 0u;

static RxState g_rx_state = RX_PREAMBLE_0;
static uint8_t g_rx_buf[16];
static uint16_t g_rx_size = 0u;
static uint16_t g_rx_type = 0u;
static uint16_t g_rx_total = 0u;
static uint16_t g_rx_pos = 0u;
static uint16_t g_rx_checksum_acc = 0u;
static uint8_t g_rx_crc_lo = 0u;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

__weak int8_t Bridge_UsbTransmit(const uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
    return -1;
}

static uint16_t q_next(uint16_t idx)
{
    idx++;
    if (idx >= BRIDGE_QUEUE_SIZE) {
        idx = 0u;
    }
    return idx;
}

static uint16_t bsu_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0u;
    for (uint16_t i = 0u; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFFu);
}

static void rx_reset(void)
{
    g_rx_state = RX_PREAMBLE_0;
    g_rx_pos = 0u;
}

static void usb_to_can_push(uint8_t bus, uint32_t can_id, const uint8_t *data)
{
    uint16_t next = q_next(g_usb_to_can_head);
    if (next == g_usb_to_can_tail) {
        return;
    }
    g_usb_to_can_q[g_usb_to_can_head].bus = bus;
    g_usb_to_can_q[g_usb_to_can_head].can_id = can_id;
    memcpy(g_usb_to_can_q[g_usb_to_can_head].data, data, 8u);
    g_usb_to_can_head = next;
}

void Bridge_CanRxPush(uint8_t can_bus, uint32_t can_id, const uint8_t *data)
{
    uint16_t next = q_next(g_can_to_usb_head);
    if (next == g_can_to_usb_tail) {
        return;
    }
    g_can_to_usb_q[g_can_to_usb_head].bus = can_bus;
    g_can_to_usb_q[g_can_to_usb_head].can_id = can_id;
    memcpy(g_can_to_usb_q[g_can_to_usb_head].data, data, 8u);
    g_can_to_usb_head = next;
    Led_NotifyCanTx();
}

void Bridge_UsbRx(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0u; i < len; i++) {
        uint8_t b = buf[i];
        switch (g_rx_state) {
        case RX_PREAMBLE_0:
            if (b == BSU_PREAMBLE_LO) {
                g_rx_state = RX_PREAMBLE_1;
            }
            break;
        case RX_PREAMBLE_1:
            if (b == BSU_PREAMBLE_HI) {
                g_rx_state = RX_SIZE_LO;
                g_rx_checksum_acc = (uint16_t)(BSU_PREAMBLE_LO + BSU_PREAMBLE_HI);
            } else {
                g_rx_state = RX_PREAMBLE_0;
            }
            break;
        case RX_SIZE_LO:
            g_rx_size = b;
            g_rx_checksum_acc += b;
            g_rx_state = RX_SIZE_HI;
            break;
        case RX_SIZE_HI:
            g_rx_size |= (uint16_t)b << 8;
            g_rx_checksum_acc += b;
            g_rx_state = RX_TYPE_LO;
            break;
        case RX_TYPE_LO:
            g_rx_type = b;
            g_rx_checksum_acc += b;
            g_rx_state = RX_TYPE_HI;
            break;
        case RX_TYPE_HI:
            g_rx_type |= (uint16_t)b << 8;
            g_rx_checksum_acc += b;
            g_rx_state = RX_SEQ_LO;
            break;
        case RX_SEQ_LO:
            g_rx_checksum_acc += b;
            g_rx_state = RX_SEQ_HI;
            break;
        case RX_SEQ_HI:
            g_rx_checksum_acc += b;
            g_rx_total = (uint16_t)(g_rx_size - BSU_HEADER_SIZE - BSU_CHECKSUM_SIZE);
            if (g_rx_size < BSU_CAN_PKT_SIZE || g_rx_total < 12u || g_rx_total > sizeof(g_rx_buf)) {
                rx_reset();
            } else {
                g_rx_pos = 0u;
                g_rx_state = RX_BODY;
            }
            break;
        case RX_BODY:
            g_rx_buf[g_rx_pos++] = b;
            g_rx_checksum_acc += b;
            if (g_rx_pos >= g_rx_total) {
                g_rx_state = RX_CRC_LO;
            }
            break;
        case RX_CRC_LO:
            g_rx_crc_lo = b;
            g_rx_state = RX_CRC_HI;
            break;
        case RX_CRC_HI: {
            uint16_t recv_crc = (uint16_t)(g_rx_crc_lo | ((uint16_t)b << 8));
            uint16_t calc_crc = (uint16_t)(g_rx_checksum_acc & 0xFFFFu);
            if (recv_crc == calc_crc && g_rx_total >= 12u &&
                (g_rx_type == BSU_PKT_TYPE_CAN || g_rx_type == BSU_PKT_TYPE_CAN2)) {
                uint32_t can_id = (uint32_t)g_rx_buf[0] |
                                  ((uint32_t)g_rx_buf[1] << 8) |
                                  ((uint32_t)g_rx_buf[2] << 16) |
                                  ((uint32_t)g_rx_buf[3] << 24);
                uint8_t bus = (g_rx_type == BSU_PKT_TYPE_CAN2) ? 2u : 1u;
                usb_to_can_push(bus, can_id, &g_rx_buf[4]);
            }
            rx_reset();
            break;
        }
        default:
            rx_reset();
            break;
        }
    }
}

static void send_can_to_usb(void)
{
    if (g_usb_tx_busy && (HAL_GetTick() - g_usb_tx_start_tick) > 50u) {
        g_usb_tx_busy = 0u;
    }
    if (g_usb_tx_busy || g_can_to_usb_head == g_can_to_usb_tail) {
        return;
    }

    const BridgeCanFrame *f = &g_can_to_usb_q[g_can_to_usb_tail];
    uint16_t pos = 0u;
    g_usb_tx_pkt[pos++] = BSU_PREAMBLE_LO;
    g_usb_tx_pkt[pos++] = BSU_PREAMBLE_HI;
    g_usb_tx_pkt[pos++] = (uint8_t)(BSU_CAN_PKT_SIZE & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)(BSU_CAN_PKT_SIZE >> 8);
    g_usb_tx_pkt[pos++] = (f->bus == 2u) ? BSU_PKT_TYPE_CAN2 : BSU_PKT_TYPE_CAN;
    g_usb_tx_pkt[pos++] = 0u;
    g_usb_tx_pkt[pos++] = 0u;
    g_usb_tx_pkt[pos++] = 0u;
    g_usb_tx_pkt[pos++] = (uint8_t)(f->can_id & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)((f->can_id >> 8) & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)((f->can_id >> 16) & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)((f->can_id >> 24) & 0xFFu);
    memcpy(&g_usb_tx_pkt[pos], f->data, 8u);
    pos += 8u;
    uint16_t crc = bsu_checksum(g_usb_tx_pkt, pos);
    g_usb_tx_pkt[pos++] = (uint8_t)(crc & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)(crc >> 8);

    if (Bridge_UsbTransmit(g_usb_tx_pkt, pos) == 0) {
        g_can_to_usb_tail = q_next(g_can_to_usb_tail);
        g_usb_tx_busy = 1u;
        g_usb_tx_start_tick = HAL_GetTick();
    }
}

static void poll_can_rx(FDCAN_HandleTypeDef *hfdcan, uint8_t bus)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
        FDCAN_RxHeaderTypeDef rxh;
        uint8_t data[8] = {0};
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, data) != HAL_OK) {
            break;
        }
        uint32_t id = (rxh.IdType == FDCAN_EXTENDED_ID) ? (rxh.Identifier & 0x1FFFFFFFu)
                                                        : (rxh.Identifier & 0x7FFu);
        Bridge_CanRxPush(bus, id, data);
    }
}

static void send_usb_to_can(void)
{
    if (g_usb_to_can_head == g_usb_to_can_tail) {
        return;
    }

    BridgeCanFrame *f = &g_usb_to_can_q[g_usb_to_can_tail];
    FDCAN_HandleTypeDef *h = (f->bus == 2u) ? &hfdcan2 : &hfdcan1;
    if (HAL_FDCAN_GetTxFifoFreeLevel(h) == 0u) {
        return;
    }

    FDCAN_TxHeaderTypeDef txh;
    txh.Identifier = f->can_id & 0x1FFFFFFFu;
    txh.IdType = FDCAN_EXTENDED_ID;
    txh.TxFrameType = FDCAN_DATA_FRAME;
    txh.DataLength = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch = FDCAN_BRS_OFF;
    txh.FDFormat = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker = 0u;
    if (HAL_FDCAN_AddMessageToTxFifoQ(h, &txh, f->data) == HAL_OK) {
        g_usb_to_can_tail = q_next(g_usb_to_can_tail);
        Led_NotifyCanTx();
    }
}

void Bridge_Init(void)
{
    g_can_to_usb_head = g_can_to_usb_tail = 0u;
    g_usb_to_can_head = g_usb_to_can_tail = 0u;
    g_usb_tx_busy = 0u;
    rx_reset();
}

void Bridge_Process(void)
{
    poll_can_rx(&hfdcan1, 1u);
    poll_can_rx(&hfdcan2, 2u);
    send_can_to_usb();
    send_usb_to_can();
}

void Bridge_UsbTxComplete(void)
{
    g_usb_tx_busy = 0u;
}