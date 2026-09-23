/**
  ******************************************************************************
  * File Name          : imu.h
  * Description        : IMU related functions and definitions
  ******************************************************************************
  */

#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "hipnuc_j1939_parser.h"
#include "hipnuc_can_common.h"
#include "canopen_parser.h"

/* Exported types ------------------------------------------------------------*/
/** @brief IMU 数据结构体（全局共享�?*/
typedef struct {
    float acc[3];    /**< 加速度 X/Y/Z (G, 重力加速度) */
    float gyro[3];   /**< 陀螺仪 X/Y/Z (deg/s) */
    float angle[3];  /**< 欧拉�?Roll/Pitch/Yaw (deg) */
    float mag[3];    /**< 磁力�?X/Y/Z */
    float quat[4];   /**< 四元�?W/X/Y/Z */
} IMU_Data_t;

/* Exported constants --------------------------------------------------------*/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

/* Exported macro ------------------------------------------------------------*/
#define IMU_DMA_RX_BUF_SIZE  64   // DMA接收缓冲区大�?

/* Exported variables --------------------------------------------------------*/
extern IMU_Data_t imu_data;    /* IMU #1: CAN ID 9 */
extern IMU_Data_t imu_data2;   /* IMU #2: CAN ID 8 */
extern volatile uint32_t can_rx_total;
extern volatile uint32_t can_rx_known;
extern volatile uint32_t can_rx_unknown;

/* Exported functions prototypes ---------------------------------------------*/
void update_imu_data_from_frame(IMU_Data_t *imu, const can_sensor_data_t *sensor, int msg_type);
void fdcan2_rx_callback(void);
void IMU_Poll_RX(void);
void IMU_Send_Data_To_USB(void);

/* ======================================================================== */
/*  �?站姿态识�?(大腿 IMU 角度阈�?+ 施密特滞�?+ 去抖)                   */
/* ======================================================================== */

/** 姿态状�?*/
typedef enum {
    POSTURE_UNKNOWN  = 0,
    POSTURE_SITTING  = 1,  /**< 坐姿 (大腿近水�? */
    POSTURE_STANDING = 2,  /**< 站姿 (大腿近竖�? */
} PostureState_t;

/** �?站切换事�?*/
typedef enum {
    POSTURE_EVT_NONE     = 0,  /**< 无事�?*/
    POSTURE_EVT_STAND_UP = 1,  /**< �?�?�?(起立) */
    POSTURE_EVT_SIT_DOWN = 2,  /**< �?�?�?(坐下) */
} PostureEvent_t;

extern PostureState_t posture_state;
extern PostureEvent_t posture_event;
extern float posture_knee_tilt_deg;
extern float posture_knee_rate_dps;

/** �?站识�?(主循环调�? */
void DetectPosture(void);


#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
