/**
 ******************************************************************************
 * @file    exo_telemetry.c
 * @brief   IMU and foot-pressure telemetry over the upper-computer CAN link.
 ******************************************************************************
 */

#include "exo_telemetry.h"
#include "robot_comm.h"
#include "imu.h"
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

    for (i = 0U; i < 3U; i++) {
        t.imu_1_acc[i]   = imu_data.acc[i];
        t.imu_1_gyro[i]  = imu_data.gyro[i];
        t.imu_1_euler[i] = imu_data.angle[i];
        t.imu_2_acc[i]   = imu_data2.acc[i];
        t.imu_2_gyro[i]  = imu_data2.gyro[i];
        t.imu_2_euler[i] = imu_data2.angle[i];
    }
    for (i = 0U; i < 4U; i++) {
        t.imu_1_quat[i] = imu_data.quat[i];
        t.imu_2_quat[i] = imu_data2.quat[i];
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
