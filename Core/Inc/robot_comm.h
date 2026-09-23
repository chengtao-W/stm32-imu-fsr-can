#ifndef __ROBOT_COMM_H
#define __ROBOT_COMM_H

#include <stdbool.h>
#include <stdint.h>

#define FRAME_HEADER1       0xEB
#define FRAME_HEADER2       0x90
#define FRAME_HEADER_LEN    4U
#define FRAME_CRC_LEN       2U
#define FRAME_OVERHEAD      (FRAME_HEADER_LEN + FRAME_CRC_LEN)

enum {
    MSG_TELEM_SENSOR_STATE = 0x05,
};


#define COMM_TX_BUFFER_SIZE      4096U
#define COMM_CAN_RX_BUFFER_SIZE  512U

/* CAN1 transport IDs for the upper-computer link. */
#define COMM_CAN_HOST_TO_MCU_ID  0x620U
#define COMM_CAN_MCU_TO_HOST_ID  0x621U

#pragma pack(push, 1)

typedef struct {
    uint32_t timestamp_ms;
    float    imu_1_acc[3];
    float    imu_1_gyro[3];
    float    imu_1_euler[3];
    float    imu_1_quat[4];
    float    imu_2_acc[3];
    float    imu_2_gyro[3];
    float    imu_2_euler[3];
    float    imu_2_quat[4];
    uint8_t  fsr_L_phase;
    uint8_t  fsr_R_phase;
    uint8_t  fsr_L_points;
    uint8_t  fsr_R_points;
    uint16_t fsr_L_total;
    uint16_t fsr_R_total;
    uint16_t fsr_L_raw[20];
    uint16_t fsr_R_raw[20];
} SensorTelemetry_t;

typedef char SensorTelemetry_Size_Must_Be_196_Bytes[
    (sizeof(SensorTelemetry_t) == 196U) ? 1 : -1];

#pragma pack(pop)

#define FRAME_MAX_PAYLOAD_LEN  ((uint8_t)sizeof(SensorTelemetry_t))
#define FRAME_MAX_LEN          (FRAME_OVERHEAD + sizeof(SensorTelemetry_t))
typedef struct {
    uint8_t buffer[COMM_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} CommRingBuffer_t;

typedef void (*CommCmdHandler)(const void *payload, uint8_t len);
typedef void (*CommAsciiSink)(uint8_t byte);

void comm_init(void);
void comm_send(uint8_t msg_id, const void *payload, uint8_t len);
void comm_flush(void);
void comm_feed_byte(uint8_t byte);
bool comm_process_can_frame(uint32_t id, uint32_t id_type,
                            const uint8_t *data, uint8_t len);
void comm_register_handler(uint8_t msg_id, CommCmdHandler handler);
void comm_register_ascii_sink(CommAsciiSink sink);

void Robot_Comm_Init(void);
void Robot_Flush_TxBuffer(void);
void Robot_Parse_Command(uint8_t *buf, uint32_t len);

#endif
