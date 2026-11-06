/**
 ******************************************************************************
 * @file    exo_telemetry.c
 * @brief   IMU and foot-pressure telemetry over the upper-computer CAN link.
 ******************************************************************************
 */

#include "exo_telemetry.h"
#include "robot_comm.h"
#include "imu.h"
#include "imu_freshness.h"
#include "fsr.h"
#include <string.h>

void ExoTelemetry_Init(void)
{
    /* Sensor-only transport: no command handlers are registered. */
}

void ExoTelemetry_Push(void)
{
    SensorTelemetry_t t;
    uint8_t i;

    memset(&t, 0, sizeof(t));
    t.timestamp_ms = HAL_GetTick();

    for (i = 0U; i < 2U; i++) {
        const IMU_Data_t *source = (i == 0U) ? &imu_data : &imu_data2;
        float *acc = (i == 0U) ? t.imu_1_acc : t.imu_2_acc;
        float *gyro = (i == 0U) ? t.imu_1_gyro : t.imu_2_gyro;
        float *euler = (i == 0U) ? t.imu_1_euler : t.imu_2_euler;
        float *quat = (i == 0U) ? t.imu_1_quat : t.imu_2_quat;
        if (!IMU_IsFresh(i, t.timestamp_ms)) {
            continue;
        }
        memcpy(acc, source->acc, sizeof(source->acc));
        memcpy(gyro, source->gyro, sizeof(source->gyro));
        memcpy(euler, source->angle, sizeof(source->angle));
        if (IMU_IsQuaternionFresh(i, t.timestamp_ms)) {
            memcpy(quat, source->quat, sizeof(source->quat));
        }
    }

    t.fsr_L_phase  = (uint8_t)fsr_data.left.phase;
    t.fsr_R_phase  = (uint8_t)fsr_data.right.phase;
    t.fsr_L_points = fsr_data.left.num_points;
    t.fsr_R_points = fsr_data.right.num_points;
    t.fsr_L_total  = fsr_data.left.total_pressure;
    t.fsr_R_total  = fsr_data.right.total_pressure;
    for (i = 0U; i < FSR_MAX_POINTS; i++) {
        t.fsr_L_raw[i] = fsr_data.left.raw[i];
        t.fsr_R_raw[i] = fsr_data.right.raw[i];
    }

    comm_send(MSG_TELEM_SENSOR_STATE, &t, sizeof(t));
}
