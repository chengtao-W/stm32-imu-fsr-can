/**
  ******************************************************************************
  * File Name          : imu.c
  * Description        : IMU related functions implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bsp_fdcan.h"
#include "canopen_parser.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
IMU_Data_t imu_data  = {0};   /* IMU #1: CAN ID 9 */
IMU_Data_t imu_data2 = {0};   /* IMU #2: CAN ID 8 */

/* CAN 甯ц鏁板櫒 (璋冭瘯鐢? */
volatile uint32_t can_rx_total   = 0;
volatile uint32_t can_rx_known   = 0;
volatile uint32_t can_rx_unknown = 0;

/* 鏈€鍚庢敹鍒扮殑鍘熷 CAN 甯?(璋冭瘯鐢? */
volatile uint32_t raw_can_id    = 0;
volatile uint32_t raw_can_id_type = 0;
volatile uint8_t  raw_can_dlc   = 0;
volatile uint8_t  raw_can_data[8] = {0};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static hipnuc_can_frame_t make_common_frame(uint32_t rec_id, uint32_t id_type, const uint8_t *data, uint8_t len)
{
    hipnuc_can_frame_t frame;

    /*
     * 灏?HAL 渚х殑甯ф牸寮忕粺涓€杞崲鎴?hipnuc 閫氱敤甯ф牸寮忋€?     * 杩欐牱鍚庣画鐨?J1939 / CANopen 瑙ｆ瀽鍣ㄥ彲浠ュ叡鐢ㄥ悓涓€濂楄緭鍏ョ粨鏋勩€?     */
    if (id_type == FDCAN_EXTENDED_ID)
    {
        frame.can_id = HIPNUC_CAN_EFF_FLAG | (rec_id & HIPNUC_CAN_EFF_MASK);
    }
    else
    {
        frame.can_id = (rec_id & HIPNUC_CAN_SFF_MASK);
    }

    if (len > sizeof(frame.data))
    {
        len = sizeof(frame.data);
    }

    frame.can_dlc = len;
    for (uint8_t i = 0; i < len; i++)
    {
        frame.data[i] = data[i];
    }
    frame.hw_ts_us = 0;
    return frame;
}
void update_imu_data_from_frame(IMU_Data_t *imu, const can_sensor_data_t *sensor, int msg_type)
{
    if (sensor == NULL || imu == NULL) return;

    if (msg_type == CAN_MSG_ACCEL || msg_type == CAN_MSG_CANFD0)
    {
        imu->acc[0] = sensor->acc_x;
        imu->acc[1] = sensor->acc_y;
        imu->acc[2] = sensor->acc_z;
    }

    if (msg_type == CAN_MSG_GYRO || msg_type == CAN_MSG_CANFD0)
    {
        imu->gyro[0] = sensor->gyr_x;
        imu->gyro[1] = sensor->gyr_y;
        imu->gyro[2] = sensor->gyr_z;
    }

    if (msg_type == CAN_MSG_EULER || msg_type == CAN_MSG_CANFD0)
    {
        imu->angle[0] = sensor->roll;
        imu->angle[1] = sensor->pitch;
        imu->angle[2] = sensor->imu_yaw;
    }
    else if (msg_type == CAN_MSG_PITCH_ROLL)
    {
        imu->angle[0] = sensor->roll;
        imu->angle[1] = sensor->pitch;
    }
    else if (msg_type == CAN_MSG_YAW)
    {
        imu->angle[2] = sensor->imu_yaw;
    }

    if (msg_type == CAN_MSG_QUAT)
    {
        imu->quat[0] = sensor->quat_w;
        imu->quat[1] = sensor->quat_x;
        imu->quat[2] = sensor->quat_y;
        imu->quat[3] = sensor->quat_z;
    }

    if (msg_type == CAN_MSG_MAG)
    {
        imu->mag[0] = sensor->mag_x;
        imu->mag[1] = sensor->mag_y;
        imu->mag[2] = sensor->mag_z;
    }
}
/**
 * @brief  杞 FDCAN2 RX FIFO锛岃В鏋愬苟鏇存柊 IMU 鏁版嵁
 * @note   鍦ㄤ富寰幆涓皟鐢紝鏇夸唬涓柇鏂瑰紡銆? *         鐩存帴璇诲瘎瀛樺櫒鍒ゆ柇 FIFO 鏄惁鏈夋暟鎹紝缁曡繃 HAL 鐘舵€佹鏌ャ€? */
void IMU_Poll_RX(void)
{
    uint32_t rec_id = 0;
    uint32_t id_type = FDCAN_STANDARD_ID;
    uint8_t rx_data[64];
    uint8_t data_len = 0;
    hipnuc_can_frame_t frame;
    can_sensor_data_t sensor;
    int msg_type;

    /* Drain RX FIFO0 while fill level > 0 */
    while ((hfdcan2.Instance->RXF0S & FDCAN_RXF0S_F0FL) != 0U)
    {
        memset(&sensor, 0, sizeof(sensor));

        data_len = fdcanx_receive_ex(&hfdcan2, &rec_id, &id_type, rx_data, &data_len);
        if (data_len == 0)
        {
            break;
        }

        can_rx_total++;

        /* save raw frame for JSON debug output */
        raw_can_id      = rec_id;
        raw_can_id_type = id_type;
        raw_can_dlc     = data_len;
        for (int _i = 0; _i < 8 && _i < data_len; _i++)
            raw_can_data[_i] = rx_data[_i];

        frame = make_common_frame(rec_id, id_type, rx_data, data_len);
        if (frame.can_id & HIPNUC_CAN_EFF_FLAG)
        {
            msg_type = hipnuc_j1939_parse_frame(&frame, &sensor);
        }
        else
        {
            msg_type = canopen_parse_frame(&frame, &sensor);
        }

        if (msg_type == CAN_MSG_ERROR || msg_type == CAN_MSG_UNKNOWN)
        {
            can_rx_unknown++;
            continue;
        }

        can_rx_known++;

        /* Route to correct IMU by CAN node ID */
        {
            uint8_t node_id;
            if (id_type == FDCAN_EXTENDED_ID)
                node_id = (uint8_t)(rec_id & 0xFF);       /* J1939 SA */
            else
                node_id = (uint8_t)(rec_id & 0x7F);       /* CANopen Node-ID */

            if (node_id == 9)
                update_imu_data_from_frame(&imu_data, &sensor, msg_type);
            else if (node_id == 8)
                update_imu_data_from_frame(&imu_data2, &sensor, msg_type);
            /* other IDs ignored */
        }
    }
}

/* ======================================================================== */
/*  鍧?绔欏Э鎬佽瘑鍒?                                                          */
/* ======================================================================== */

PostureState_t posture_state = POSTURE_UNKNOWN;
PostureEvent_t posture_event = POSTURE_EVT_NONE;
float posture_knee_tilt_deg = 0.0f;
float posture_knee_rate_dps = 0.0f;

#define KNEE_STAND_TILT_DEG      30.0f
#define KNEE_SIT_TILT_DEG        55.0f
#define KNEE_STAND_RATE_DPS     -12.0f
#define KNEE_SIT_RATE_DPS        12.0f
#define KNEE_TILT_LPF_ALPHA       0.25f
#define KNEE_POSTURE_DT_S         0.10f
#define KNEE_POSTURE_DEBOUNCE     3U
#define KNEE_IMU_VALID_MIN_DEG    0.5f

static float knee_tilt_from_imu(const IMU_Data_t *imu)
{
    float roll = fabsf(imu->angle[0]);
    float pitch = fabsf(imu->angle[1]);
    return (roll > pitch) ? roll : pitch;
}

static uint8_t knee_imu_is_valid(const IMU_Data_t *imu)
{
    float roll = fabsf(imu->angle[0]);
    float pitch = fabsf(imu->angle[1]);
    float yaw = fabsf(imu->angle[2]);
    float acc_sum = fabsf(imu->acc[0]) + fabsf(imu->acc[1]) + fabsf(imu->acc[2]);
    return ((roll + pitch + yaw) > KNEE_IMU_VALID_MIN_DEG) || (acc_sum > 0.05f);
}

void DetectPosture(void)
{
    static float filt_tilt = 0.0f;
    static float prev_filt_tilt = 0.0f;
    static uint8_t filt_init = 0U;
    static PostureState_t last_candidate = POSTURE_UNKNOWN;
    static uint8_t debounce_cnt = 0U;

    float tilt_sum = 0.0f;
    uint8_t valid_count = 0U;

    if (knee_imu_is_valid(&imu_data)) {
        tilt_sum += knee_tilt_from_imu(&imu_data);
        valid_count++;
    }
    if (knee_imu_is_valid(&imu_data2)) {
        tilt_sum += knee_tilt_from_imu(&imu_data2);
        valid_count++;
    }

    posture_event = POSTURE_EVT_NONE;
    if (valid_count == 0U) {
        return;
    }

    float raw_tilt = tilt_sum / (float)valid_count;
    if (!filt_init) {
        filt_tilt = raw_tilt;
        prev_filt_tilt = raw_tilt;
        filt_init = 1U;
    } else {
        filt_tilt += KNEE_TILT_LPF_ALPHA * (raw_tilt - filt_tilt);
    }

    posture_knee_tilt_deg = filt_tilt;
    posture_knee_rate_dps = (filt_tilt - prev_filt_tilt) / KNEE_POSTURE_DT_S;
    prev_filt_tilt = filt_tilt;

    PostureState_t candidate = posture_state;
    if (filt_tilt <= KNEE_STAND_TILT_DEG) {
        candidate = POSTURE_SITTING;
    } else if (filt_tilt >= KNEE_SIT_TILT_DEG) {
        candidate = POSTURE_STANDING;
    } else if (posture_state == POSTURE_SITTING && posture_knee_rate_dps > KNEE_SIT_RATE_DPS) {
        candidate = POSTURE_STANDING;
    } else if (posture_state == POSTURE_STANDING && posture_knee_rate_dps < KNEE_STAND_RATE_DPS) {
        candidate = POSTURE_SITTING;
    } else if (posture_state == POSTURE_UNKNOWN) {
        candidate = (filt_tilt > ((KNEE_STAND_TILT_DEG + KNEE_SIT_TILT_DEG) * 0.5f)) ? POSTURE_STANDING : POSTURE_SITTING;
    }

    if (candidate == last_candidate) {
        if (debounce_cnt < KNEE_POSTURE_DEBOUNCE) debounce_cnt++;
    } else {
        last_candidate = candidate;
        debounce_cnt = 0U;
    }

    if (debounce_cnt >= KNEE_POSTURE_DEBOUNCE && candidate != posture_state) {
        if (candidate == POSTURE_STANDING && posture_state == POSTURE_SITTING) {
            posture_event = POSTURE_EVT_STAND_UP;
        } else if (candidate == POSTURE_SITTING && posture_state == POSTURE_STANDING) {
            posture_event = POSTURE_EVT_SIT_DOWN;
        }

        posture_state = candidate;
        printf("[POSTURE] %s tilt=%.1f rate=%.1f\r\n",
               (candidate == POSTURE_STANDING) ? "STANDING" : "SITTING",
               posture_knee_tilt_deg,
               posture_knee_rate_dps);
    }
}
void fdcan2_rx_callback(void)
{
    /*
     * CAN2 涓柇鍥炶皟 (ISR context)锛?     * 鍦ㄥ惊鐜腑璇诲彇 FIFO 涓殑鎵€鏈夊抚锛岄槻姝腑鏂粎瑙﹀彂涓€娆?     * 瀵艰嚧 FIFO 鍫嗙Н銆?     *
     * 娉ㄦ剰锛氭湰鍑芥暟鍦?ISR 涓墽琛岋紝绂佹璋冪敤 printf 绛夐樆濉炴搷浣溿€?     */
    uint32_t rec_id = 0;
    uint32_t id_type = FDCAN_STANDARD_ID;
    uint8_t rx_data[64] = {0};
    uint8_t data_len = 0;
    hipnuc_can_frame_t frame;
    can_sensor_data_t sensor;
    int msg_type;

    /*
     * Drain ALL pending frames from RX FIFO0.
     * The watermark IRQ may only fire once even if multiple frames
     * are queued 鈥?reading in a loop guarantees we catch them all.
     */
    while (1)
    {
        /* Reset sensor data to zeros before each parse */
        memset(&sensor, 0, sizeof(sensor));

        data_len = fdcanx_receive_ex(&hfdcan2, &rec_id, &id_type, rx_data, &data_len);
        if (data_len == 0)
        {
            break;  /* FIFO empty */
        }

        can_rx_total++;

        /* save raw frame for JSON debug output */
        raw_can_id      = rec_id;
        raw_can_id_type = id_type;
        raw_can_dlc     = data_len;
        for (int _i = 0; _i < 8 && _i < data_len; _i++)
            raw_can_data[_i] = rx_data[_i];

        frame = make_common_frame(rec_id, id_type, rx_data, data_len);
        if (frame.can_id & HIPNUC_CAN_EFF_FLAG)
        {
            msg_type = hipnuc_j1939_parse_frame(&frame, &sensor);
        }
        else
        {
            msg_type = canopen_parse_frame(&frame, &sensor);
        }

        if (msg_type == CAN_MSG_ERROR || msg_type == CAN_MSG_UNKNOWN)
        {
            can_rx_unknown++;
            continue;  /* try next frame in FIFO */
        }

        can_rx_known++;
        {
            uint8_t node_id;
            if (id_type == FDCAN_EXTENDED_ID)
                node_id = (uint8_t)(rec_id & 0xFF);
            else
                node_id = (uint8_t)(rec_id & 0x7F);

            if (node_id == 9)
                update_imu_data_from_frame(&imu_data, &sensor, msg_type);
            else if (node_id == 8)
                update_imu_data_from_frame(&imu_data2, &sensor, msg_type);
        }
    }
}
/* End of file. */
