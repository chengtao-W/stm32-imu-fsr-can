/**
 ******************************************************************************
 * @file    fsr.h
 * @brief   墨现足底压力传感�?(FSR) 驱动 �?USB Host CDC 传输
 *
 * @details 协议: 0xA5 0x5A + CMD(1B) + LEN(2B LE) + DATA + CRC16(2B LE)
 *          CRC16: Modbus 多项�?0xA001
 *          硬件:  STM32 USB OTG (Host) �?墨现传感�?(CDC ACM)
 *          数据�? USBH_CDC_ReceiveCallback �?FSR_ProcessUSBData()
 *
 *          参�? matrix_insole_heatmap_20points.m (墨现官方 MATLAB 例程)
 *          LEN = CMD(1) + LEN(2) + DATA + CRC(2), �?SOF 之后的所有字�? ******************************************************************************
 */

#ifndef __FSR_H
#define __FSR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================================================================== */
/*  传感器网格定�?(5�?× 4�?= 20 个传感点, 对齐 MATLAB 例程)               */
/*                                                                          */
/*  �?0 (后跟):    [ 0] [ 1] [ 2] [ 3]                                     */
/*  �?1 (足弓�?:  [ 4] [ 5] [ 6] [ 7]                                     */
/*  �?2 (足弓�?:  [ 8] [ 9] [10] [11]                                     */
/*  �?3 (跖骨):    [12] [13] [14] [15]                                     */
/*  �?4 (脚尖):    [16] [17] [18] [19]                                     */
/*                                                                          */
/*  参�?MATLAB:                                                             */
/*   heatmap_matrix(1,1:4) = adcValues(1:4);    % 后跟                      */
/*   heatmap_matrix(2,1:4) = adcValues(5:8);    % 足弓�?                   */
/*   heatmap_matrix(3,1:4) = adcValues(9:12);   % 足弓�?                   */
/*   heatmap_matrix(4,1:4) = adcValues(13:16);  % 跖骨                      */
/*   heatmap_matrix(5,1:4) = adcValues(17:20);  % 脚尖                      */
/* ======================================================================== */

#define FSR_MAX_POINTS      20
#define FSR_GRID_COLS       4
#define FSR_GRID_ROWS       5

/** 步态检测区域索引范�?*/
#define FSR_IDX_HEEL_START   0
#define FSR_IDX_HEEL_END     3     /* �?0: 4 个传感器 */
#define FSR_IDX_ARCH_START   4
#define FSR_IDX_ARCH_END    11     /* �?1-2 合并: 8 个传感器 */
#define FSR_IDX_META_START  12
#define FSR_IDX_META_END    15     /* �?3: 4 个传感器 */
#define FSR_IDX_TOE_START   16
#define FSR_IDX_TOE_END     19     /* �?4: 4 个传感器 */

/* ======================================================================== */
/*  数据类型定义                                                             */
/* ======================================================================== */

/** 步态相�?*/
/* ======================================================================== */
/*  Moxian insole routing mode                                               */
/* ======================================================================== */

typedef enum {
    FSR_MODE_SINGLE_LEFT  = 0,
    FSR_MODE_SINGLE_RIGHT = 1,
    FSR_MODE_DUAL_ALT     = 2,
} FSR_Mode_t;
typedef enum {
    GAIT_SWING         = 0,  /**< 摆动�?(足离�?            */
    GAIT_HEEL_STRIKE   = 1,  /**< 后跟着�?(初始接触)        */
    GAIT_MID_STANCE    = 2,  /**< 支撑中期 (全掌着�?        */
    GAIT_PUSH_OFF      = 3,  /**< 蹬离�?(前掌发力)          */
} GaitPhase_t;

/** 单足压力数据 */
typedef struct {
    uint16_t raw[FSR_MAX_POINTS];  /**< 原始 ADC �?(0~255)   */
    uint8_t  num_points;           /**< 实际传感器点�?         */
    uint16_t total_pressure;       /**< 总压�?(各点之和)      */
    GaitPhase_t phase;             /**< 当前检测到的步态相�?   */
    bool      contact;             /**< 是否接触地面            */
    bool      fresh;               /**< 本周期是否有新数�?     */
} FSR_Foot_t;

/** 双足 FSR 数据 (全局) */
typedef struct {
    FSR_Foot_t left;               /**< 左脚鞋垫                */
    FSR_Foot_t right;              /**< 右脚鞋垫                */
    uint32_t  frame_count;         /**< 累计接收到有效帧�?     */
    uint32_t  crc_errors;          /**< CRC 校验错误计数 (调试) */
} FSR_Data_t;

/* ======================================================================== */
/*  全局变量                                                                 */
/* ======================================================================== */

extern FSR_Data_t fsr_data;

/* ======================================================================== */
/*  公开 API                                                                 */
/* ======================================================================== */

/** 初始�?FSR 数据结构与解析器状�?*/
void FSR_Init(void);

/**
 * @brief  处理�?USB Host CDC 管道接收到的原始字节
 * @note   �?USBH_CDC_ReceiveCallback() 调用, 在回调上下文中安�? * @param  data  接收缓冲区指�? * @param  len   缓冲区中的字节数
 */
void FSR_ProcessUSBData(const uint8_t *data, uint16_t len);

/**
 * @brief  发�?开始采�?命令 (通过 USB Host CDC)
 * @param  phost  USB Host 句柄
 */
void FSR_SendStartCapture(void *phost);

/**
 * @brief  根据足底压力数据检测步态相�? * @note   在主循环中调�? 带施密特滞回�?3 帧确�? */
void FSR_DetectGaitPhase(void);
void FSR_SetMode(FSR_Mode_t mode);
FSR_Mode_t FSR_GetMode(void);
const char *FSR_ModeName(FSR_Mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __FSR_H */
