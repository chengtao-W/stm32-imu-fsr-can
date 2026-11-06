#ifndef __IMU_FRESHNESS_H
#define __IMU_FRESHNESS_H

#include <stdbool.h>
#include <stdint.h>

/* Index 0 is IMU node 9; index 1 is IMU node 8. */
bool IMU_IsFresh(uint8_t index, uint32_t now_ms);
bool IMU_IsQuaternionFresh(uint8_t index, uint32_t now_ms);

#endif
