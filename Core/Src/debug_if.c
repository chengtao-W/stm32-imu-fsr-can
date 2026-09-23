/**
 ******************************************************************************
 * @file    debug_if.c
 * @brief   调试�?�?运行时参数整�?ASCII 命令接口
 *
 * 从通信�?(robot_comm.c) 拆出。通过 comm_register_ascii_sink() 接入 UART1 RX,
 * 接收非二进制帧的 ASCII 字节, 解析命令行并整定 dbg_cfg / 执行动作�? *
 * 传感器命令: HELP, STATUS, FSR LEFT|RIGHT|DUAL|STATUS
 ******************************************************************************
 */

#include "debug_if.h"
#include "main.h"
#include "fsr.h"
#include <stdio.h>
#include <string.h>

/* ======================================================================== */
/*  全局调试配置                                                             */
/* ======================================================================== */

DebugConfig_t dbg_cfg;

/* ======================================================================== */
/*  行缓�?                                                                   */
/* ======================================================================== */

#define DBG_LINE_BUF_SIZE 64
static uint8_t  dbg_line_buf[DBG_LINE_BUF_SIZE];
static uint16_t dbg_line_idx = 0;

static int dbg_stricmp(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = (*a >= 'A' && *a <= 'Z') ? (*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (*b + 32) : *b;
        if (ca != cb) return (int)(ca - cb);
        a++; b++;
    }
    return (int)(*a - *b);
}

static void DebugIf_ProcessLine(const char *line);

/* ======================================================================== */
/*  初始�?                                                                  */
/* ======================================================================== */

void DebugIf_Init(void)
{
    dbg_cfg.fsr_contact_on    = 80;
    dbg_cfg.fsr_contact_off   = 30;
    dbg_cfg.fsr_heel_thresh   = 100;
    dbg_line_idx = 0;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  CtrBoard Sensor CAN System\r\n");
    printf("========================================\r\n");
    printf("  STATUS    -- sensor status\r\n");
    printf("  FSR LEFT|RIGHT|DUAL|STATUS\r\n");
    printf("  HELP      -- command list\r\n");
    printf("========================================\r\n");
}

/* ======================================================================== */
/*  字节馈入                                                                 */
/* ======================================================================== */

void DebugIf_FeedByte(uint8_t byte)
{
    if (byte == 0) return;
    if (byte == '\n' || byte == '\r') {
        if (dbg_line_idx > 0) {
            dbg_line_buf[dbg_line_idx] = '\0';
            DebugIf_ProcessLine((const char *)dbg_line_buf);
            dbg_line_idx = 0;
        }
        return;
    }
    if (byte == 0x08 || byte == 0x7F) {
        if (dbg_line_idx > 0) dbg_line_idx--;
        return;
    }
    if (dbg_line_idx < DBG_LINE_BUF_SIZE - 1) {
        dbg_line_buf[dbg_line_idx++] = byte;
    }
}

/* ======================================================================== */
/*  状态打�?                                                                */
/* ======================================================================== */

void DebugIf_PrintStatus(void)
{
    printf("[SENSOR] FSR mode=%s frames=%lu crc=%lu L=%u R=%u Lph=%d Rph=%d\r\n",
           FSR_ModeName(FSR_GetMode()),
           fsr_data.frame_count, fsr_data.crc_errors,
           fsr_data.left.total_pressure, fsr_data.right.total_pressure,
           (int)fsr_data.left.phase, (int)fsr_data.right.phase);
}

/* ======================================================================== */
/*  命令行解�?                                                              */
/* ======================================================================== */

static void DebugIf_ProcessLine(const char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    char cmd[16] = {0}, arg[32] = {0};
    sscanf(line, "%15s %31s", cmd, arg);

    #define CMD_IS(s)  (dbg_stricmp(cmd, s) == 0)

    if (CMD_IS("HELP") || CMD_IS("?")) {
        printf("STATUS FSR LEFT|RIGHT|DUAL|STATUS HELP\r\n");
    }
    else if (CMD_IS("STATUS")) {
        DebugIf_PrintStatus();
    }
    else if (CMD_IS("FSR")) {
        if (dbg_stricmp(arg, "LEFT") == 0 || dbg_stricmp(arg, "L") == 0) {
            FSR_SetMode(FSR_MODE_SINGLE_LEFT);
        } else if (dbg_stricmp(arg, "RIGHT") == 0 || dbg_stricmp(arg, "R") == 0) {
            FSR_SetMode(FSR_MODE_SINGLE_RIGHT);
        } else if (dbg_stricmp(arg, "DUAL") == 0 || dbg_stricmp(arg, "D") == 0) {
            FSR_SetMode(FSR_MODE_DUAL_ALT);
        } else if (arg[0] != '\0' && dbg_stricmp(arg, "STATUS") != 0) {
            printf("[FSR] Usage: FSR LEFT|RIGHT|DUAL|STATUS\r\n");
            return;
        }
        printf("[FSR] mode=%s frames=%lu crc=%lu L=%u R=%u Lph=%d Rph=%d\r\n",
               FSR_ModeName(FSR_GetMode()),
               fsr_data.frame_count, fsr_data.crc_errors,
               fsr_data.left.total_pressure, fsr_data.right.total_pressure,
               (int)fsr_data.left.phase, (int)fsr_data.right.phase);
    }
    else {
        printf("ERR: unknown '%s'\r\n", cmd);
    }
    #undef CMD_IS
}


