#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cmd/cmd_arbiter.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Placeholder for CRSF/SBUS → motion_cmd_t. */
void rc_link_init(void);
void rc_link_poll(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
