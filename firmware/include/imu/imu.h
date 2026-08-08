#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float pitch_rad;
    float pitch_rate_radps;
    float yaw_rate_radps;
    uint32_t stamp_ms;
    bool valid;
} imu_sample_t;

bool imu_init(void);
bool imu_update(imu_sample_t *out);
const imu_sample_t *imu_latest(void);

#ifdef __cplusplus
}
#endif
