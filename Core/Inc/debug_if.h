/**
 ******************************************************************************
 * @file    debug_if.h
 * @brief   运行时调试 / 参数整定接口 (UART1 ASCII 文本命令)
 *
 * @details 与 robot_comm (二进制协议) 共享 UART1 RX。
 *          命令格式: CMD [参数1] [参数2] ... \r\n
 *          响应格式: "OK\r\n" 或 "ERR: 原因\r\n"
 *
 *          传感器命令: HELP, STATUS, FSR LEFT|RIGHT|DUAL|STATUS
 ******************************************************************************
 */

#ifndef __DEBUG_IF_H
#define __DEBUG_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================================================================== */
/*  可调参数集 (全部运行时可变, 无需重新编译)                                */
/* ======================================================================== */

typedef struct {
    uint16_t fsr_contact_on;  /**< 接触 ON  阈值              默认 80   */
    uint16_t fsr_contact_off; /**< 接触 OFF 阈值              默认 30   */
    uint16_t fsr_heel_thresh; /**< 后跟触发阈值               默认 100  */
} DebugConfig_t;

/** 足底压力检测阈值。 */
extern DebugConfig_t dbg_cfg;

/* ======================================================================== */
/*  公开 API                                                                 */
/* ======================================================================== */

/** 初始化调试配置 (设为默认值) */
void DebugIf_Init(void);

/**
 * @brief  向文本命令行解析器馈送一个接收字节
 * @note   在 UART1 RX 回调中调用 (与二进制协议解析并存)
 * @param  byte  接收到的字符
 */
void DebugIf_FeedByte(uint8_t byte);

/** 输出当前传感器状态 (响应 STATUS 命令) */
void DebugIf_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_IF_H */

