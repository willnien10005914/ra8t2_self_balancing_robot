#include "imu/lsm6dsk320x.h"

#include <string.h>

/*
 * Stub / skeleton. Wire R_IIC_Master_Write/Read from FSP in the port section.
 * Register map must be verified against ST LSM6DSK320X datasheet (WHO_AM_I etc).
 */

#define REG_WHO_AM_I   (0x0Fu)
#define REG_CTRL1_XL   (0x10u)
#define REG_CTRL2_G    (0x11u)
#define REG_OUTX_L_G   (0x22u)

static uint8_t s_addr = 0x6A;
static bool s_ready;

/* Weak port hooks — replace in board file or override with FSP calls. */
__attribute__((weak)) bool imu_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint32_t len)
{
    (void)addr7; (void)reg; (void)data; (void)len;
    return false;
}

__attribute__((weak)) bool imu_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint32_t len)
{
    (void)addr7; (void)reg; (void)data; (void)len;
    return false;
}

uint8_t lsm6dsk320x_who_am_i(void)
{
    uint8_t id = 0;
    (void)imu_i2c_read(s_addr, REG_WHO_AM_I, &id, 1);
    return id;
}

bool lsm6dsk320x_init(uint8_t i2c_addr7)
{
    s_addr = i2c_addr7;
    /* Example bring-up: enable XL + G at high ODR — tune with datasheet. */
    uint8_t xl = 0x60; /* placeholder ODR/FS */
    uint8_t g  = 0x60;
    s_ready = imu_i2c_write(s_addr, REG_CTRL1_XL, &xl, 1) &&
              imu_i2c_write(s_addr, REG_CTRL2_G, &g, 1);
    return s_ready;
}

bool lsm6dsk320x_read_raw(int16_t accel[3], int16_t gyro[3])
{
    uint8_t raw[12];
    if (!s_ready || !imu_i2c_read(s_addr, REG_OUTX_L_G, raw, 12))
    {
        return false;
    }
    gyro[0] = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    gyro[1] = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    gyro[2] = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
    accel[0] = (int16_t)((uint16_t)raw[7] << 8 | raw[6]);
    accel[1] = (int16_t)((uint16_t)raw[9] << 8 | raw[8]);
    accel[2] = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);
    return true;
}
