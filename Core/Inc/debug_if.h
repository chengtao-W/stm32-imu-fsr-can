/**
 ******************************************************************************
 * @file    debug_if.h
 * @brief   运行时调试 / 参数整定接口 (UART1 ASCII 文本命令)
 *
 * @details 与 robot_comm (二进制协议) 共享 UART1 RX。
 *          命令格式: CMD [参数1] [参数2] ... \r\n
 *          响应格式: "OK\r\n" 或 "ERR: 原因\r\n"
 *
 *          传感器网关命令: HELP, STATUS, FSR LEFT|RIGHT|DUAL|STATUS
 ******************************************************************************
 */

#ifndef __DEBUG_IF_H
#define __DEBUG_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================== */
/*  可调参数集 (全部运行时可变, 无需重新编译)                                */
/* ======================================================================== */

typedef struct {
    /* ---- 阻抗控制器参数 ---- */
    float kp_hip;            /**< 刚度         (Nm/rad),      默认 10.0  */
    float kd_hip;            /**< 阻尼         (Nm/(rad/s)),  默认  0.5  */
    float ki_hip;            /**< 积分增益 (备用)              默认 0.05  */
    float max_torque;        /**< 扭矩上限     (Nm),          默认  7.0  */

    /* ---- AO 自适应振荡器参数 ---- */
    float ao_omega_init;     /**< 初始频率     (Hz),          默认  1.0  */
    float ao_epsilon;        /**< 输入耦合强度                 默认  1.0  */
    float ao_nu;             /**< 频率学习率                   默认  2.0  */
    float ao_omega_min;      /**< 频率下限     (Hz),          默认  0.3  */
    float ao_omega_max;      /**< 频率上限     (Hz),          默认  3.5  */

    /* ---- 步态轨迹参数 ---- */
    float swing_amplitude;   /**< 摆动幅度     (rad),         默认  0.5  */
    float joint_offset;      /**< 关节偏置角   (rad),         默认  1.0  */

    /* ---- FSR 步态检测参数 ---- */
    uint16_t fsr_contact_on;  /**< 接触 ON  阈值              默认 80   */
    uint16_t fsr_contact_off; /**< 接触 OFF 阈值              默认 30   */
    uint16_t fsr_heel_thresh; /**< 后跟触发阈值               默认 100  */

    /* ---- 使能开关 ---- */
    bool ctrl_enabled;        /**< 控制总使能 (OFF = 急停)    默认 true  */
    bool motor1_enabled;      /**< 左髋电机使能               默认 true  */
    bool motor2_enabled;      /**< 右髋电机使能               默认 true  */
    bool telemetry_enabled;   /**< ROS/PC 二进制遥测输出       默认 false */

    /* ---- 相位复位 ---- */
    uint8_t blend_cycles;     /**< 软混合周期数                默认 5    */

    /* ---- AO 测试 ---- */
    float ao_test_freq;       /**< 合成正弦波频率 (Hz), 0=关闭   默认 0   */
} DebugConfig_t;

/** 全局调试配置 (控制回路读取, 调试接口写入) */
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

/** 输出当前全部参数表 (响应 STATUS 命令) */
void DebugIf_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_IF_H */

