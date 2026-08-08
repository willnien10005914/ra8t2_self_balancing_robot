#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Low-level LSM6DSK320X register access over FSP I2C (port in .c). */
bool lsm6dsk320x_init(uint8_t i2c_addr7);
bool lsm6dsk320x_read_raw(int16_t accel[3], int16_t gyro[3]);
uint8_t lsm6dsk320x_who_am_i(void);

#ifdef __cplusplus
}
#endif
