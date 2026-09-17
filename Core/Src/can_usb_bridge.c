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
#define BSU_PKT_MAX_SIZE   (BSU_HEADER_SIZE + ESP_UART_BODY_MAX + BSU_CHECKSUM_SIZE)

#define RS_BUS_PREAMBLE_0  0xA5u
#define RS_BUS_PREAMBLE_1  0x5Au

#define BRIDGE_QUEUE_SIZE  128u
#define RS_QUEUE_SIZE      16u
#define RS_RX_BUF_SIZE     288u

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t bus; /* 1=CAN1, 2=CAN2 */
} BridgeCanFrame;

typedef struct {
    uint16_t len;
    uint8_t data[ESP_UART_BODY_MAX];
} BridgeRsFrame;

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

static BridgeRsFrame g_rs_to_usb_q[RS_QUEUE_SIZE];
static volatile uint16_t g_rs_to_usb_head = 0u;
static volatile uint16_t g_rs_to_usb_tail = 0u;

static BridgeRsFrame g_usb_to_rs_q[RS_QUEUE_SIZE];
static volatile uint16_t g_usb_to_rs_head = 0u;
static volatile uint16_t g_usb_to_rs_tail = 0u;

static uint8_t g_usb_tx_pkt[BSU_PKT_MAX_SIZE];
static volatile uint8_t g_usb_tx_busy = 0u;
static uint32_t g_usb_tx_start_tick = 0u;
static uint8_t g_usb_tx_prefer_rs = 0u;
static uint16_t g_esp_uart_seq = 0u;

static RxState g_rx_state = RX_PREAMBLE_0;
static uint8_t g_rx_buf[ESP_UART_BODY_MAX];
static uint16_t g_rx_size = 0u;
static uint16_t g_rx_type = 0u;
static uint16_t g_rx_total = 0u;
static uint16_t g_rx_pos = 0u;
static uint16_t g_rx_checksum_acc = 0u;
static uint8_t g_rx_crc_lo = 0u;

static uint8_t g_rs_uart_rx[RS_RX_BUF_SIZE];
static uint8_t g_rs_parse_buf[RS_RX_BUF_SIZE];
static uint16_t g_rs_parse_len = 0u;
static volatile uint8_t g_rs_tx_busy = 0u;
static uint32_t g_rs_tx_start_tick = 0u;
static uint8_t g_rs_tx_copy[ESP_UART_BODY_MAX];
static uint16_t g_rs_tx_len = 0u;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern UART_HandleTypeDef huart4;

__weak int8_t Bridge_UsbTransmit(const uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
    return -1;
}

static uint16_t q_next(uint16_t idx, uint16_t size)
{
    idx++;
    if (idx >= size) {
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

static uint16_t rs_checksum16(const uint8_t *data, uint16_t len)
{
    return bsu_checksum(data, len);
}

static void rx_reset(void)
{
    g_rx_state = RX_PREAMBLE_0;
    g_rx_pos = 0u;
}

static void rs_rx_arm(void)
{
    (void)HAL_UART_AbortReceive(&huart4);
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart4, g_rs_uart_rx, sizeof(g_rs_uart_rx));
}

static void usb_to_can_push(uint8_t bus, uint32_t can_id, const uint8_t *data)
{
    uint16_t next = q_next(g_usb_to_can_head, BRIDGE_QUEUE_SIZE);
    if (next == g_usb_to_can_tail) {
        return;
    }
    g_usb_to_can_q[g_usb_to_can_head].bus = bus;
    g_usb_to_can_q[g_usb_to_can_head].can_id = can_id;
    memcpy(g_usb_to_can_q[g_usb_to_can_head].data, data, 8u);
    g_usb_to_can_head = next;
}

static void usb_to_rs_push(const uint8_t *data, uint16_t len)
{
    uint16_t next;
    if (data == 0 || len == 0u || len > ESP_UART_BODY_MAX) {
        return;
    }
    next = q_next(g_usb_to_rs_head, RS_QUEUE_SIZE);
    if (next == g_usb_to_rs_tail) {
        return;
    }
    g_usb_to_rs_q[g_usb_to_rs_head].len = len;
    memcpy(g_usb_to_rs_q[g_usb_to_rs_head].data, data, len);
    g_usb_to_rs_head = next;
}

static void rs_to_usb_push(const uint8_t *data, uint16_t len)
{
    uint16_t next;
    if (data == 0 || len == 0u || len > ESP_UART_BODY_MAX) {
        return;
    }
    next = q_next(g_rs_to_usb_head, RS_QUEUE_SIZE);
    if (next == g_rs_to_usb_tail) {
        return;
    }
    g_rs_to_usb_q[g_rs_to_usb_head].len = len;
    memcpy(g_rs_to_usb_q[g_rs_to_usb_head].data, data, len);
    g_rs_to_usb_head = next;
    Led_NotifyCanTx();
}

static uint8_t rs_frame_try_decode(const uint8_t *src, uint16_t src_size,
                                   uint16_t *out_consumed)
{
    uint16_t len_field;
    uint16_t total_size;
    uint16_t rx_crc;
    uint16_t calc_crc;

    if (out_consumed != 0) {
        *out_consumed = 0u;
    }
    if (src == 0 || src_size < 9u) {
        return 0u;
    }
    if (src[0] != RS_BUS_PREAMBLE_0 || src[1] != RS_BUS_PREAMBLE_1) {
        return 0u;
    }

    len_field = src[2];
    if (len_field < 4u) {
        return 0u;
    }
    total_size = (uint16_t)(2u + 1u + len_field + 2u);
    if (src_size < total_size) {
        return 0u;
    }

    rx_crc = (uint16_t)src[(uint16_t)(3u + len_field)] |
             (uint16_t)((uint16_t)src[(uint16_t)(4u + len_field)] << 8);
    calc_crc = rs_checksum16(&src[3], len_field);
    if (rx_crc != calc_crc) {
        return 0u;
    }
    if (out_consumed != 0) {
        *out_consumed = total_size;
    }
    return 1u;
}

static void rs_process_rx_bytes(const uint8_t *data, uint16_t len)
{
    uint16_t pos;
    uint16_t consumed;

    if (data == 0 || len == 0u) {
        return;
    }
    if ((uint32_t)g_rs_parse_len + len > sizeof(g_rs_parse_buf)) {
        g_rs_parse_len = 0u;
    }
    memcpy(&g_rs_parse_buf[g_rs_parse_len], data, len);
    g_rs_parse_len = (uint16_t)(g_rs_parse_len + len);

    pos = 0u;
    while (pos < g_rs_parse_len) {
        if (g_rs_parse_buf[pos] != RS_BUS_PREAMBLE_0) {
            pos++;
            continue;
        }
        if ((uint16_t)(g_rs_parse_len - pos) < 2u ||
            g_rs_parse_buf[(uint16_t)(pos + 1u)] != RS_BUS_PREAMBLE_1) {
            break;
        }
        if (!rs_frame_try_decode(&g_rs_parse_buf[pos],
                                 (uint16_t)(g_rs_parse_len - pos),
                                 &consumed)) {
            if ((uint16_t)(g_rs_parse_len - pos) < 9u) {
                break;
            }
            pos++;
            continue;
        }
        if (consumed > 0u && consumed <= ESP_UART_BODY_MAX) {
            rs_to_usb_push(&g_rs_parse_buf[pos], consumed);
        }
        pos = (uint16_t)(pos + consumed);
    }

    if (pos != 0u) {
        memmove(g_rs_parse_buf, &g_rs_parse_buf[pos], (size_t)(g_rs_parse_len - pos));
        g_rs_parse_len = (uint16_t)(g_rs_parse_len - pos);
    }
}

void Bridge_CanRxPush(uint8_t can_bus, uint32_t can_id, const uint8_t *data)
{
    uint16_t next = q_next(g_can_to_usb_head, BRIDGE_QUEUE_SIZE);
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
            if (g_rx_size < (BSU_HEADER_SIZE + BSU_CHECKSUM_SIZE + 1u)) {
                rx_reset();
                break;
            }
            g_rx_total = (uint16_t)(g_rx_size - BSU_HEADER_SIZE - BSU_CHECKSUM_SIZE);
            if (g_rx_total == 0u || g_rx_total > sizeof(g_rx_buf)) {
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
            if (recv_crc == calc_crc) {
                if ((g_rx_type == BSU_PKT_TYPE_CAN || g_rx_type == BSU_PKT_TYPE_CAN2) &&
                    g_rx_total >= 12u) {
                    uint32_t can_id = (uint32_t)g_rx_buf[0] |
                                      ((uint32_t)g_rx_buf[1] << 8) |
                                      ((uint32_t)g_rx_buf[2] << 16) |
                                      ((uint32_t)g_rx_buf[3] << 24);
                    uint8_t bus = (g_rx_type == BSU_PKT_TYPE_CAN2) ? 2u : 1u;
                    usb_to_can_push(bus, can_id, &g_rx_buf[4]);
                } else if (g_rx_type == BSU_PKT_TYPE_ESP_UART &&
                           g_rx_total > 0u && g_rx_total <= ESP_UART_BODY_MAX) {
                    usb_to_rs_push(g_rx_buf, g_rx_total);
                }
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

static int send_bsu_can_to_usb(const BridgeCanFrame *f)
{
    uint16_t pos = 0u;
    uint16_t crc;

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
    crc = bsu_checksum(g_usb_tx_pkt, pos);
    g_usb_tx_pkt[pos++] = (uint8_t)(crc & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)(crc >> 8);

    return Bridge_UsbTransmit(g_usb_tx_pkt, pos);
}

static int send_bsu_rs_to_usb(const BridgeRsFrame *f)
{
    uint16_t pkt_size = (uint16_t)(BSU_HEADER_SIZE + f->len + BSU_CHECKSUM_SIZE);
    uint16_t pos = 0u;
    uint16_t crc;
    uint16_t seq = g_esp_uart_seq++;

    g_usb_tx_pkt[pos++] = BSU_PREAMBLE_LO;
    g_usb_tx_pkt[pos++] = BSU_PREAMBLE_HI;
    g_usb_tx_pkt[pos++] = (uint8_t)(pkt_size & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)(pkt_size >> 8);
    g_usb_tx_pkt[pos++] = (uint8_t)(BSU_PKT_TYPE_ESP_UART & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)(BSU_PKT_TYPE_ESP_UART >> 8);
    g_usb_tx_pkt[pos++] = (uint8_t)(seq & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)(seq >> 8);
    memcpy(&g_usb_tx_pkt[pos], f->data, f->len);
    pos = (uint16_t)(pos + f->len);
    crc = bsu_checksum(g_usb_tx_pkt, pos);
    g_usb_tx_pkt[pos++] = (uint8_t)(crc & 0xFFu);
    g_usb_tx_pkt[pos++] = (uint8_t)(crc >> 8);

    return Bridge_UsbTransmit(g_usb_tx_pkt, pos);
}

static void send_to_usb(void)
{
    uint8_t can_pending;
    uint8_t rs_pending;
    uint8_t send_rs;

    if (g_usb_tx_busy && (HAL_GetTick() - g_usb_tx_start_tick) > 50u) {
        g_usb_tx_busy = 0u;
    }
    if (g_usb_tx_busy) {
        return;
    }

    can_pending = (g_can_to_usb_head != g_can_to_usb_tail) ? 1u : 0u;
    rs_pending = (g_rs_to_usb_head != g_rs_to_usb_tail) ? 1u : 0u;
    if (!can_pending && !rs_pending) {
        return;
    }

    if (can_pending && rs_pending) {
        send_rs = g_usb_tx_prefer_rs;
        g_usb_tx_prefer_rs ^= 1u;
    } else {
        send_rs = rs_pending;
    }

    if (send_rs) {
        const BridgeRsFrame *f = &g_rs_to_usb_q[g_rs_to_usb_tail];
        if (send_bsu_rs_to_usb(f) == 0) {
            g_rs_to_usb_tail = q_next(g_rs_to_usb_tail, RS_QUEUE_SIZE);
            g_usb_tx_busy = 1u;
            g_usb_tx_start_tick = HAL_GetTick();
        }
    } else {
        const BridgeCanFrame *f = &g_can_to_usb_q[g_can_to_usb_tail];
        if (send_bsu_can_to_usb(f) == 0) {
            g_can_to_usb_tail = q_next(g_can_to_usb_tail, BRIDGE_QUEUE_SIZE);
            g_usb_tx_busy = 1u;
            g_usb_tx_start_tick = HAL_GetTick();
        }
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
        g_usb_to_can_tail = q_next(g_usb_to_can_tail, BRIDGE_QUEUE_SIZE);
        Led_NotifyCanTx();
    }
}

static void send_usb_to_rs(void)
{
    BridgeRsFrame *f;

    if (g_rs_tx_busy) {
        if ((HAL_GetTick() - g_rs_tx_start_tick) > 20u) {
            HAL_NVIC_DisableIRQ(UART4_IRQn);
            (void)HAL_UART_AbortTransmit(&huart4);
            g_rs_tx_busy = 0u;
            g_rs_parse_len = 0u;
            rs_rx_arm();
            HAL_NVIC_EnableIRQ(UART4_IRQn);
        }
        return;
    }
    if (g_usb_to_rs_head == g_usb_to_rs_tail) {
        return;
    }

    f = &g_usb_to_rs_q[g_usb_to_rs_tail];
    g_rs_tx_len = f->len;
    memcpy(g_rs_tx_copy, f->data, f->len);

    HAL_NVIC_DisableIRQ(UART4_IRQn);
    (void)HAL_UART_AbortReceive(&huart4);
    if (HAL_UART_Transmit_IT(&huart4, g_rs_tx_copy, g_rs_tx_len) == HAL_OK) {
        g_rs_tx_busy = 1u;
        g_rs_tx_start_tick = HAL_GetTick();
        g_usb_to_rs_tail = q_next(g_usb_to_rs_tail, RS_QUEUE_SIZE);
        Led_NotifyCanTx();
    } else {
        rs_rx_arm();
    }
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}

void Bridge_UartRxEvent(uint16_t size)
{
    if (g_rs_tx_busy == 0u && size > 0u) {
        rs_process_rx_bytes(g_rs_uart_rx, size);
    }
    if (g_rs_tx_busy == 0u) {
        (void)HAL_UARTEx_ReceiveToIdle_IT(&huart4, g_rs_uart_rx, sizeof(g_rs_uart_rx));
    }
}

void Bridge_UartTxCplt(void)
{
    uint32_t spins = 0u;
    while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET) {
        spins++;
        if (spins > 2000000u) {
            break;
        }
    }
    g_rs_tx_busy = 0u;
    g_rs_parse_len = 0u;
    rs_rx_arm();
}

void Bridge_UartError(void)
{
    /* Во время TX ошибки RX/abort не трогаем: завершит TxCplt или таймаут. */
    if (g_rs_tx_busy != 0u) {
        return;
    }
    g_rs_parse_len = 0u;
    rs_rx_arm();
}

void Bridge_Init(void)
{
    g_can_to_usb_head = g_can_to_usb_tail = 0u;
    g_usb_to_can_head = g_usb_to_can_tail = 0u;
    g_rs_to_usb_head = g_rs_to_usb_tail = 0u;
    g_usb_to_rs_head = g_usb_to_rs_tail = 0u;
    g_usb_tx_busy = 0u;
    g_rs_tx_busy = 0u;
    g_rs_parse_len = 0u;
    g_esp_uart_seq = 0u;
    g_usb_tx_prefer_rs = 0u;
    rx_reset();
    rs_rx_arm();
}

void Bridge_Process(void)
{
    poll_can_rx(&hfdcan1, 1u);
    poll_can_rx(&hfdcan2, 2u);
    send_to_usb();
    send_usb_to_can();
    send_usb_to_rs();
}

void Bridge_UsbTxComplete(void)
{
    g_usb_tx_busy = 0u;
}
