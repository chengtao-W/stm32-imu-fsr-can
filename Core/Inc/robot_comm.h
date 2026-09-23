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
    MSG_CMD_JOINT_TARGET   = 0x01,
    MSG_TELEM_JOINT_STATE  = 0x02,
    MSG_CMD_CONTROL        = 0x03,
    MSG_TELEM_EXO_STATE    = 0x04,
    MSG_TELEM_SENSOR_STATE = 0x05,
    MSG_TELEM_IMU1         = 0x10,
    MSG_TELEM_IMU2         = 0x11,
};

enum {
    CMD_SET_MODE   = 1,
    CMD_SET_ENABLE = 2,
    CMD_ESTOP      = 3,
    CMD_SET_PARAM  = 4,
};

enum {
    PARAM_KP = 0, PARAM_KD, PARAM_KI, PARAM_TORQ,
    PARAM_AMP, PARAM_OFFSET, PARAM_FREQ, PARAM_EPS, PARAM_NU,
    PARAM_BLEND, PARAM_HEEL,
};


#define COMM_TX_BUFFER_SIZE      4096U
#define COMM_CAN_RX_BUFFER_SIZE  512U

/* CAN1 transport IDs. Keep these away from the motor IDs (0x004/0x005). */
#define COMM_CAN_HOST_TO_MCU_ID  0x620U
#define COMM_CAN_MCU_TO_HOST_ID  0x621U
#define FLAG_CTRL_ENABLED    0x01
#define FLAG_MOTOR1_ENABLED  0x02
#define FLAG_MOTOR2_ENABLED  0x04
#define FLAG_OUTPUT_ALLOWED  0x08
#define FLAG_SAFETY_FAULT    0x10

#pragma pack(push, 1)

typedef struct {
    float position;
    float velocity;
    float effort;
} JointState_t;

typedef struct {
    JointState_t joints[6];
} Payload_t;

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

typedef struct {
    uint8_t cmd;
    uint8_t mode;
    uint8_t enable;
    uint8_t param_id;
    float   value;
} ControlCmd_t;

typedef union {
    Payload_t      joints;
    SensorTelemetry_t sensors;
    ControlCmd_t   control;
} FramePayload_u;

typedef struct {
    uint8_t  header1;
    uint8_t  header2;
    uint8_t  msg_id;
    uint8_t  length;
    FramePayload_u payload;
    uint16_t crc16;
} CommFrame_t;

#pragma pack(pop)

#define FRAME_MAX_PAYLOAD_LEN  ((uint8_t)sizeof(FramePayload_u))
#define FRAME_MAX_LEN          (FRAME_OVERHEAD + sizeof(FramePayload_u))
typedef struct {
    uint8_t buffer[COMM_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} CommRingBuffer_t;

typedef void (*CommCmdHandler)(const void *payload, uint8_t len);
typedef void (*CommAsciiSink)(uint8_t byte);

extern Payload_t target_cmd;

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
