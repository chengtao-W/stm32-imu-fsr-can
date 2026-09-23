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
/** @brief IMU data shared by acquisition and telemetry tasks. */
typedef struct {
    float acc[3];    /**< Acceleration X/Y/Z (g). */
    float gyro[3];   /**< Angular velocity X/Y/Z (deg/s). */
    float angle[3];  /**< Euler angle Roll/Pitch/Yaw (deg). */
    float mag[3];    /**< Magnetic field X/Y/Z. */
    float quat[4];   /**< Quaternion W/X/Y/Z. */
} IMU_Data_t;

/* Exported constants --------------------------------------------------------*/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

/* Exported macro ------------------------------------------------------------*/
#define IMU_DMA_RX_BUF_SIZE  64

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

/* ======================================================================== */
/*  Sit/stand recognition using thigh angle, hysteresis and debounce.        */
/* ======================================================================== */

/** Posture state. */
typedef enum {
    POSTURE_UNKNOWN  = 0,
    POSTURE_SITTING  = 1,
    POSTURE_STANDING = 2,
} PostureState_t;

/** Posture transition event. */
typedef enum {
    POSTURE_EVT_NONE     = 0,
    POSTURE_EVT_STAND_UP = 1,
    POSTURE_EVT_SIT_DOWN = 2,
} PostureEvent_t;

extern PostureState_t posture_state;
extern PostureEvent_t posture_event;
extern float posture_knee_tilt_deg;
extern float posture_knee_rate_dps;

/** Update posture recognition; call from the main loop. */
void DetectPosture(void);


#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
