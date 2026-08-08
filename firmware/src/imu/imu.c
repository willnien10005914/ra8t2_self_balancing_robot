#include "imu/imu.h"
#include "imu/lsm6dsk320x.h"
#include "util/util.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static imu_sample_t s_latest;
static float s_pitch;
static bool s_ok;

bool imu_init(void)
{
    memset(&s_latest, 0, sizeof(s_latest));
    s_pitch = 0.0f;
    s_ok = lsm6dsk320x_init(0x6A);
    return s_ok;
}

const imu_sample_t *imu_latest(void)
{
    return &s_latest;
}

bool imu_update(imu_sample_t *out)
{
    int16_t a[3];
    int16_t g[3];

    if (!s_ok || !lsm6dsk320x_read_raw(a, g))
    {
        s_latest.valid = false;
        if (out) *out = s_latest;
        return false;
    }

    /* Sensitivity placeholders — match LSM6DSK320X FS config in driver. */
    const float acc_s = 0.000061f;   /* g/LSB @ ~2g */
    const float gyr_s = 0.01750f;    /* dps/LSB @ ~500dps approx */

    s_latest.ax_g = a[0] * acc_s;
    s_latest.ay_g = a[1] * acc_s;
    s_latest.az_g = a[2] * acc_s;
    s_latest.gx_dps = g[0] * gyr_s;
    s_latest.gy_dps = g[1] * gyr_s;
    s_latest.gz_dps = g[2] * gyr_s;

    const float pitch_acc = atan2f(-s_latest.ax_g, sqrtf(s_latest.ay_g * s_latest.ay_g +
                                                         s_latest.az_g * s_latest.az_g));
    const float gyro_y = s_latest.gy_dps * ((float)M_PI / 180.0f);
    const float dt = 0.001f; /* refine with actual period */
    const float alpha = 0.98f;
    s_pitch = alpha * (s_pitch + gyro_y * dt) + (1.0f - alpha) * pitch_acc;

    s_latest.pitch_rad = s_pitch;
    s_latest.pitch_rate_radps = gyro_y;
    s_latest.yaw_rate_radps = s_latest.gz_dps * ((float)M_PI / 180.0f);
    s_latest.stamp_ms = time_ms_get();
    s_latest.valid = true;

    if (out)
    {
        *out = s_latest;
    }
    return true;
}
