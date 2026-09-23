/**
 ******************************************************************************
 * @file    fsr.c
 * @brief   Moxian 20-point insole parser over USB Host CDC.
 ******************************************************************************
 */

#include "fsr.h"
#include "debug_if.h"
#include "usbh_cdc.h"
#include <stdio.h>
#include <string.h>

#define FSR_SOF1         0xA5U
#define FSR_SOF2         0x5AU
#define FSR_CMD_DATA     0x01U
#define FSR_CMD_ACK      0x02U
#define FSR_BUF_SIZE     128U
#define FSR_FRAME_TAG    0x01U
#define FSR_PRINT_FIRST  3U
#define FSR_PRINT_STEP   100U

typedef enum {
    FSR_STATE_SOF1 = 0,
    FSR_STATE_SOF2,
    FSR_STATE_CMD,
    FSR_STATE_LEN1,
    FSR_STATE_LEN2,
    FSR_STATE_DATA,
    FSR_STATE_CRC1,
    FSR_STATE_CRC2,
} fsr_state_t;

FSR_Data_t fsr_data = {0};

static FSR_Mode_t fsr_mode = FSR_MODE_SINGLE_LEFT;
static fsr_state_t state = FSR_STATE_SOF1;
static uint8_t  cmd = 0;
static uint16_t data_len = 0;
static uint16_t data_index = 0;
static uint8_t  data_buf[FSR_BUF_SIZE];
static uint16_t received_crc = 0;
static uint8_t  crc_buf[FSR_BUF_SIZE + 5U];
static uint16_t crc_index = 0;

static void       FSR_ResetParser(void);
static uint16_t   FSR_CRC16(const uint8_t *data, uint16_t length);
static void       FSR_ParseByte(uint8_t byte);
static void       FSR_HandleFrame(uint8_t frame_cmd, const uint8_t *data, uint16_t len);
static FSR_Foot_t *FSR_SelectTarget(void);
static uint16_t   FSR_UpdateFoot(FSR_Foot_t *foot, const uint8_t *data, uint8_t n);

void FSR_Init(void)
{
    memset(&fsr_data, 0, sizeof(fsr_data));
    FSR_ResetParser();
}

void FSR_ProcessUSBData(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return;
    }
    for (uint16_t i = 0; i < len; i++) {
        FSR_ParseByte(data[i]);
    }
}

void FSR_SetMode(FSR_Mode_t mode)
{
    if (mode > FSR_MODE_DUAL_ALT) {
        mode = FSR_MODE_SINGLE_LEFT;
    }

    fsr_mode = mode;
    memset(&fsr_data.left, 0, sizeof(fsr_data.left));
    memset(&fsr_data.right, 0, sizeof(fsr_data.right));
    fsr_data.frame_count = 0;
}

FSR_Mode_t FSR_GetMode(void)
{
    return fsr_mode;
}

const char *FSR_ModeName(FSR_Mode_t mode)
{
    switch (mode) {
    case FSR_MODE_SINGLE_LEFT:  return "LEFT";
    case FSR_MODE_SINGLE_RIGHT: return "RIGHT";
    case FSR_MODE_DUAL_ALT:     return "DUAL";
    default:                    return "UNKNOWN";
    }
}

void FSR_SendStartCapture(void *phost)
{
    USBH_HandleTypeDef *host = (USBH_HandleTypeDef *)phost;
    uint8_t cmd_buf[16];
    uint8_t idx = 0;

    if (host == NULL) {
        return;
    }

    cmd_buf[idx++] = FSR_SOF1;
    cmd_buf[idx++] = FSR_SOF2;
    cmd_buf[idx++] = FSR_CMD_ACK;
    cmd_buf[idx++] = 0x09U;
    cmd_buf[idx++] = 0x00U;
    cmd_buf[idx++] = 0x01U;
    cmd_buf[idx++] = 0x01U;
    cmd_buf[idx++] = 0x01U;
    cmd_buf[idx++] = 0x01U;

    uint16_t crc = FSR_CRC16(cmd_buf, idx);
    cmd_buf[idx++] = (uint8_t)(crc & 0xFFU);
    cmd_buf[idx++] = (uint8_t)(crc >> 8);

    printf("[FSR] Start-capture cmd:");
    for (uint8_t i = 0; i < idx; i++) {
        printf(" %02X", cmd_buf[i]);
    }
    printf(" (CRC=0x%04X)\r\n", crc);

    if (USBH_CDC_Transmit(host, cmd_buf, idx) != USBH_OK) {
        printf("[FSR] ERROR: start-capture TX failed\r\n");
    }
}

void FSR_DetectGaitPhase(void)
{
    FSR_Foot_t *feet[2] = { &fsr_data.left, &fsr_data.right };
    static GaitPhase_t last_cand[2] = { GAIT_SWING, GAIT_SWING };
    static uint8_t deb_cnt[2] = { 0U, 0U };
    const uint8_t phase_debounce_cnt = 3U;

    for (int f = 0; f < 2; f++) {
        FSR_Foot_t *ft = feet[f];
        uint16_t heel_sum = 0;
        uint16_t arch_sum = 0;
        uint16_t meta_sum = 0;
        uint16_t toe_sum = 0;

        for (int i = FSR_IDX_HEEL_START; i <= FSR_IDX_HEEL_END; i++) heel_sum += ft->raw[i];
        for (int i = FSR_IDX_ARCH_START; i <= FSR_IDX_ARCH_END; i++) arch_sum += ft->raw[i];
        for (int i = FSR_IDX_META_START; i <= FSR_IDX_META_END; i++) meta_sum += ft->raw[i];
        for (int i = FSR_IDX_TOE_START; i <= FSR_IDX_TOE_END; i++) toe_sum += ft->raw[i];

        uint16_t total = heel_sum + arch_sum + meta_sum + toe_sum;
        uint16_t forefoot = toe_sum + meta_sum;
        ft->total_pressure = total;

        if (!ft->contact && total > dbg_cfg.fsr_contact_on) {
            ft->contact = true;
        } else if (ft->contact && total < dbg_cfg.fsr_contact_off) {
            ft->contact = false;
        }

        GaitPhase_t candidate;
        if (!ft->contact) {
            candidate = GAIT_SWING;
        } else if (heel_sum > (forefoot * 2U) && heel_sum > dbg_cfg.fsr_heel_thresh) {
            candidate = GAIT_HEEL_STRIKE;
        } else if (forefoot > (heel_sum * 2U) && forefoot > dbg_cfg.fsr_heel_thresh) {
            candidate = GAIT_PUSH_OFF;
        } else {
            candidate = GAIT_MID_STANCE;
        }

        if (candidate == last_cand[f]) {
            if (deb_cnt[f] < phase_debounce_cnt) {
                deb_cnt[f]++;
            }
            if (deb_cnt[f] >= phase_debounce_cnt) {
                ft->phase = candidate;
            }
        } else {
            deb_cnt[f] = 0U;
            last_cand[f] = candidate;
        }

        ft->fresh = false;
    }
}

static void FSR_ResetParser(void)
{
    state = FSR_STATE_SOF1;
    cmd = 0U;
    data_len = 0U;
    data_index = 0U;
    received_crc = 0U;
    crc_index = 0U;
}

static void FSR_ParseByte(uint8_t byte)
{
    switch (state) {
    case FSR_STATE_SOF1:
        if (byte == FSR_SOF1) {
            crc_index = 0U;
            crc_buf[crc_index++] = byte;
            state = FSR_STATE_SOF2;
        }
        break;

    case FSR_STATE_SOF2:
        if (byte == FSR_SOF2) {
            crc_buf[crc_index++] = byte;
            state = FSR_STATE_CMD;
        } else {
            FSR_ResetParser();
        }
        break;

    case FSR_STATE_CMD:
        cmd = byte;
        crc_buf[crc_index++] = byte;
        state = FSR_STATE_LEN1;
        break;

    case FSR_STATE_LEN1:
        data_len = byte;
        crc_buf[crc_index++] = byte;
        state = FSR_STATE_LEN2;
        break;

    case FSR_STATE_LEN2:
        data_len |= ((uint16_t)byte << 8);
        crc_buf[crc_index++] = byte;
        data_index = 0U;

        if (data_len == 21U) {
            /* Older format: LEN is payload length. */
        } else if (data_len > 5U && data_len <= (FSR_BUF_SIZE + 5U)) {
            data_len -= 5U;
        } else {
            FSR_ResetParser();
            break;
        }

        if (data_len == 0U || data_len > FSR_BUF_SIZE) {
            FSR_ResetParser();
        } else {
            state = FSR_STATE_DATA;
        }
        break;

    case FSR_STATE_DATA:
        data_buf[data_index++] = byte;
        crc_buf[crc_index++] = byte;
        if (data_index >= data_len) {
            state = FSR_STATE_CRC1;
        }
        break;

    case FSR_STATE_CRC1:
        received_crc = byte;
        state = FSR_STATE_CRC2;
        break;

    case FSR_STATE_CRC2: {
        received_crc |= ((uint16_t)byte << 8);
        uint16_t calc_crc = FSR_CRC16(crc_buf, crc_index);
        if (calc_crc == received_crc) {
            FSR_HandleFrame(cmd, data_buf, data_len);
        } else {
            fsr_data.crc_errors++;
            if (fsr_data.crc_errors <= 5U || (fsr_data.crc_errors % 100U) == 0U) {
                printf("[FSR] CRC err #%lu: calc=0x%04X recv=0x%04X cmd=0x%02X len=%u\r\n",
                       fsr_data.crc_errors, calc_crc, received_crc, cmd, data_len);
            }
        }
        FSR_ResetParser();
        break;
    }

    default:
        FSR_ResetParser();
        break;
    }
}

static uint16_t FSR_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8U; j++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (crc >> 1) ^ 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static FSR_Foot_t *FSR_SelectTarget(void)
{
    switch (fsr_mode) {
    case FSR_MODE_SINGLE_RIGHT:
        return &fsr_data.right;
    case FSR_MODE_DUAL_ALT:
        return (fsr_data.frame_count & 1U) ? &fsr_data.right : &fsr_data.left;
    case FSR_MODE_SINGLE_LEFT:
    default:
        return &fsr_data.left;
    }
}

static uint16_t FSR_UpdateFoot(FSR_Foot_t *foot, const uint8_t *data, uint8_t n)
{
    uint16_t sum = 0U;

    foot->num_points = n;
    for (uint8_t i = 0; i < FSR_MAX_POINTS; i++) {
        uint16_t v = (i < n) ? data[1U + i] : 0U;
        foot->raw[i] = v;
        sum += v;
    }
    foot->total_pressure = sum;
    foot->fresh = true;
    return sum;
}

static void FSR_HandleFrame(uint8_t frame_cmd, const uint8_t *data, uint16_t len)
{
    if (frame_cmd == FSR_CMD_DATA && len >= 2U) {
        uint8_t n = (uint8_t)(len - 1U);
        FSR_Foot_t *target;

        if (data[0] != FSR_FRAME_TAG) {
            return;
        }
        if (n > FSR_MAX_POINTS) {
            n = FSR_MAX_POINTS;
        }

        target = FSR_SelectTarget();
        if (target != NULL && n > 0U) {
            uint16_t sum = FSR_UpdateFoot(target, data, n);
            fsr_data.frame_count++;

            if (fsr_data.frame_count <= FSR_PRINT_FIRST ||
                (fsr_data.frame_count % FSR_PRINT_STEP) == 0U) {
                printf("[FSR] frame=%lu mode=%s foot=%s sum=%u pts=%u\r\n",
                       fsr_data.frame_count,
                       FSR_ModeName(fsr_mode),
                       (target == &fsr_data.left) ? "L" : "R",
                       sum, n);
            }
        }
    } else if (frame_cmd == FSR_CMD_ACK) {
        printf("[FSR] ACK received\r\n");
    }
}
