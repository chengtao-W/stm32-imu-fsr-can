#include "robot_comm.h"
#include "usart.h"
#include "fdcan.h"
#include "bsp_fdcan.h"
#include "debug_if.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define COMM_HANDLER_COUNT  256U
CommRingBuffer_t tx_ring_buffer = {0};

static uint8_t rx_byte;
static CommCmdHandler handlers[COMM_HANDLER_COUNT];
static CommAsciiSink ascii_sink;

static uint8_t can_rx_buffer[COMM_CAN_RX_BUFFER_SIZE];
static volatile uint16_t can_rx_head;
static volatile uint16_t can_rx_tail;
static volatile uint16_t can_rx_count;
static uint32_t can_tx_last_tick;

static uint8_t rx_state;
static uint8_t rx_msg_id;
static uint8_t rx_payload_len;
static uint8_t rx_payload_idx;
static uint8_t rx_payload[FRAME_MAX_PAYLOAD_LEN];
static uint8_t rx_header[FRAME_HEADER_LEN];
static uint16_t rx_crc;

static uint16_t comm_crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void comm_reset_rx(void)
{
    rx_state = 0;
    rx_msg_id = 0;
    rx_payload_len = 0;
    rx_payload_idx = 0;
    rx_crc = 0;
}

static void comm_arm_rx(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

static bool comm_tx_push_byte(uint8_t byte)
{
    if (tx_ring_buffer.count >= COMM_TX_BUFFER_SIZE) {
        return false;
    }
    tx_ring_buffer.buffer[tx_ring_buffer.head] = byte;
    tx_ring_buffer.head = (uint16_t)((tx_ring_buffer.head + 1U) % COMM_TX_BUFFER_SIZE);
    tx_ring_buffer.count++;
    return true;
}

static void comm_process_pending_rx(void)
{
    uint8_t byte;
    uint16_t budget = 64U;

    while (budget-- > 0U) {
        __disable_irq();
        if (can_rx_count == 0U) {
            __enable_irq();
            break;
        }
        byte = can_rx_buffer[can_rx_tail];
        can_rx_tail = (uint16_t)((can_rx_tail + 1U) % COMM_CAN_RX_BUFFER_SIZE);
        can_rx_count--;
        __enable_irq();
        comm_feed_byte(byte);
    }
}

static void comm_can_rx_poll(void)
{
    uint32_t id;
    uint32_t id_type;
    uint8_t data[8];
    uint8_t len;
    uint8_t budget = 32U;

    while (budget-- > 0U &&
           (hfdcan1.Instance->RXF0S & FDCAN_RXF0S_F0FL) != 0U) {
        len = 0U;
        if (fdcanx_receive_ex(&hfdcan1, &id, &id_type, data, &len) == 0U) {
            break;
        }
        (void)comm_process_can_frame(id, id_type, data, len);
    }
}

static void comm_can_tx_step(void)
{
    uint8_t frame[8];
    uint8_t len;
    uint8_t result;
    uint16_t index;
    uint32_t now = HAL_GetTick();

    if (now == can_tx_last_tick || tx_ring_buffer.count == 0U ||
        HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        return;
    }
    can_tx_last_tick = now;

    __disable_irq();
    len = (tx_ring_buffer.count > sizeof(frame)) ? (uint8_t)sizeof(frame)
                                                  : (uint8_t)tx_ring_buffer.count;
    index = tx_ring_buffer.tail;
    for (uint8_t i = 0U; i < len; i++) {
        frame[i] = tx_ring_buffer.buffer[index];
        index = (uint16_t)((index + 1U) % COMM_TX_BUFFER_SIZE);
    }
    __enable_irq();

    /* Keep the queue update and hardware FIFO write atomic. */
    __disable_irq();
    result = fdcanx_send_data_v2(&hfdcan1, COMM_CAN_MCU_TO_HOST_ID,
                                 FDCAN_STANDARD_ID, frame, len);
    if (result == 0U) {
        tx_ring_buffer.tail = index;
        tx_ring_buffer.count = (uint16_t)(tx_ring_buffer.count - len);
    }
    __enable_irq();
}

void comm_register_handler(uint8_t msg_id, CommCmdHandler handler)
{
    handlers[msg_id] = handler;
}

void comm_register_ascii_sink(CommAsciiSink sink)
{
    ascii_sink = sink;
}

void comm_init(void)
{
    __disable_irq();
    memset((void *)&tx_ring_buffer, 0, sizeof(tx_ring_buffer));
    can_rx_head = 0U;
    can_rx_tail = 0U;
    can_rx_count = 0U;
    __enable_irq();

    memset(handlers, 0, sizeof(handlers));
    ascii_sink = DebugIf_FeedByte;
    comm_reset_rx();
    can_tx_last_tick = HAL_GetTick() - 1U;
    comm_arm_rx();
}

void comm_send(uint8_t msg_id, const void *payload, uint8_t len)
{
    if ((len > 0U && payload == NULL) || len == 0U) {
        return;
    }

    const uint8_t *bytes = (const uint8_t *)payload;
    uint8_t header[FRAME_HEADER_LEN] = {
        FRAME_HEADER1,
        FRAME_HEADER2,
        msg_id,
        len
    };
    uint16_t crc_buf_len = (uint16_t)(FRAME_HEADER_LEN + len);
    uint8_t crc_buf[FRAME_HEADER_LEN + FRAME_MAX_PAYLOAD_LEN];
    uint16_t crc;
    uint16_t required = (uint16_t)(FRAME_OVERHEAD + len);

    if (len > FRAME_MAX_PAYLOAD_LEN) {
        return;
    }

    memcpy(crc_buf, header, FRAME_HEADER_LEN);
    memcpy(&crc_buf[FRAME_HEADER_LEN], bytes, len);
    crc = comm_crc16(crc_buf, crc_buf_len);

    __disable_irq();
    if ((COMM_TX_BUFFER_SIZE - tx_ring_buffer.count) < required) {
        __enable_irq();
        return;
    }

    (void)comm_tx_push_byte(header[0]);
    (void)comm_tx_push_byte(header[1]);
    (void)comm_tx_push_byte(header[2]);
    (void)comm_tx_push_byte(header[3]);
    for (uint8_t i = 0U; i < len; i++) {
        (void)comm_tx_push_byte(bytes[i]);
    }
    (void)comm_tx_push_byte((uint8_t)(crc & 0xFFU));
    (void)comm_tx_push_byte((uint8_t)(crc >> 8));
    __enable_irq();

    comm_flush();
}

void comm_flush(void)
{
    comm_can_rx_poll();
    comm_process_pending_rx();
    comm_can_tx_step();
}

bool comm_process_can_frame(uint32_t id, uint32_t id_type,
                            const uint8_t *data, uint8_t len)
{
    if (id_type != FDCAN_STANDARD_ID || id != COMM_CAN_HOST_TO_MCU_ID) {
        return false;
    }
    if (data == NULL || len == 0U || len > 8U) {
        return true;
    }

    __disable_irq();
    if ((COMM_CAN_RX_BUFFER_SIZE - can_rx_count) >= len) {
        for (uint8_t i = 0U; i < len; i++) {
            can_rx_buffer[can_rx_head] = data[i];
            can_rx_head = (uint16_t)((can_rx_head + 1U) % COMM_CAN_RX_BUFFER_SIZE);
        }
        can_rx_count = (uint16_t)(can_rx_count + len);
    }
    __enable_irq();
    return true;
}

void comm_feed_byte(uint8_t byte)
{
    switch (rx_state) {
    case 0:
        if (byte == FRAME_HEADER1) {
            rx_header[0] = byte;
            rx_state = 1;
        } else if (ascii_sink != NULL) {
            ascii_sink(byte);
        }
        break;

    case 1:
        if (byte == FRAME_HEADER2) {
            rx_header[1] = byte;
            rx_state = 2;
        } else {
            if (ascii_sink != NULL) {
                ascii_sink(FRAME_HEADER1);
                ascii_sink(byte);
            }
            comm_reset_rx();
        }
        break;

    case 2:
        rx_msg_id = byte;
        rx_header[2] = byte;
        rx_state = 3;
        break;

    case 3:
        rx_payload_len = byte;
        rx_header[3] = byte;
        rx_payload_idx = 0;
        if (rx_payload_len > FRAME_MAX_PAYLOAD_LEN) {
            comm_reset_rx();
        } else if (rx_payload_len == 0U) {
            rx_state = 5;
        } else {
            rx_state = 4;
        }
        break;

    case 4:
        rx_payload[rx_payload_idx++] = byte;
        if (rx_payload_idx >= rx_payload_len) {
            rx_state = 5;
        }
        break;

    case 5:
        rx_crc = byte;
        rx_state = 6;
        break;

    case 6: {
        rx_crc |= (uint16_t)byte << 8;
        uint8_t crc_buf[FRAME_HEADER_LEN + FRAME_MAX_PAYLOAD_LEN];
        memcpy(crc_buf, rx_header, FRAME_HEADER_LEN);
        if (rx_payload_len > 0U) {
            memcpy(&crc_buf[FRAME_HEADER_LEN], rx_payload, rx_payload_len);
        }
        uint16_t calc = comm_crc16(crc_buf, FRAME_HEADER_LEN + rx_payload_len);
        if (calc == rx_crc && handlers[rx_msg_id] != NULL) {
            handlers[rx_msg_id](rx_payload, rx_payload_len);
        }
        comm_reset_rx();
        break;
    }

    default:
        comm_reset_rx();
        break;
    }
}

void Robot_Comm_Init(void)
{
    comm_init();
}

void Robot_Flush_TxBuffer(void)
{
    comm_flush();
}

void Robot_Parse_Command(uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len < FRAME_OVERHEAD) {
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        comm_feed_byte(buf[i]);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        DebugIf_FeedByte(rx_byte);
        comm_arm_rx();
    }
}

