#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"
#include "hipnuc_can_common.h"

#define hcan_t FDCAN_HandleTypeDef

#define CAN_CLASS   0
#define CAN_FD_BRS  1

#define CAN_BR_125K 0
#define CAN_BR_200K 1
#define CAN_BR_250K 2
#define CAN_BR_500K 3
#define CAN_BR_1M   4
#define CAN_BR_2M   5
#define CAN_BR_2M5  6
#define CAN_BR_3M2  7
#define CAN_BR_4M   8
#define CAN_BR_5M   9

void bsp_can_init(void);
void bsp_can_it_enable(void);
void can1_filter_init(void);
void can2_filter_init(void);
void can3_filter_init(void);
void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);


/* �� bsp_fdcan.h ���������º������� */
uint8_t fdcanx_send_data_v2(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint32_t id_type, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive_ex(hcan_t *hfdcan, uint32_t *rec_id, uint32_t *id_type, uint8_t *buf, uint8_t *len);

/* J1939 IMU 配置命令 (via FDCAN2) */
uint8_t hipnuc_j1939_send_cfg(FDCAN_HandleTypeDef *hfdcan, uint8_t imu_id, uint8_t src_addr,
                               uint16_t addr, uint32_t val);
void hipnuc_imu_config(void);

#endif /* __BSP_FDCAN_H_ */

