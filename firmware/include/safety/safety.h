#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_OK = 0,
    SAFETY_FAULT_TILT = 1u << 0,
    SAFETY_FAULT_IMU = 1u << 1,
    SAFETY_FAULT_UV = 1u << 2,
    SAFETY_FAULT_OV = 1u << 3,
    SAFETY_FAULT_OC = 1u << 4,
    SAFETY_FAULT_OT = 1u << 5,
    SAFETY_FAULT_MOTOR = 1u << 6,
    SAFETY_FAULT_ESTOP = 1u << 7
} safety_fault_t;

typedef struct {
    float vbus_v;
    float ibus_a;
    float temp_c;
    float i_motor_l_a;
    float i_motor_r_a;
    bool valid;
} safety_sensors_t;

void safety_init(void);
/** Sample sensors (board port) and evaluate limits; returns latching fault mask. */
uint16_t safety_update(void);
uint16_t safety_fault_mask(void);
void safety_clear_faults(void);
bool safety_ok(void);

/** Board hooks — implement with ADC / FSP. */
bool safety_port_read(safety_sensors_t *out);
/** Hardware PWM kill via POEG / break input — must work even if app loops hang. */
void safety_port_hw_shutdown(void);

#ifdef __cplusplus
}
#endif
